/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/constraint_checker/collision_checker.h"

#include <utility>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/path_matcher.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::PathMatcher;
using apollo::common::math::Vec2d;

CollisionChecker::CollisionChecker(
    const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line,
    const ReferenceLineInfo* ptr_reference_line_info,
    const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph) {
  ptr_reference_line_info_ = ptr_reference_line_info;
  ptr_path_time_graph_ = ptr_path_time_graph;
  BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d,
                            discretized_reference_line);
}

bool CollisionChecker::InCollision(
    const std::vector<const Obstacle*>& obstacles,
    const DiscretizedTrajectory& ego_trajectory, const double ego_length,
    const double ego_width, const double ego_back_edge_to_center) {
  for (size_t i = 0; i < ego_trajectory.NumOfPoints(); ++i) {
    const auto& ego_point =
        ego_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));
    const auto relative_time = ego_point.relative_time();
    const auto ego_theta = ego_point.path_point().theta();

    Box2d ego_box({ego_point.path_point().x(), ego_point.path_point().y()},
                  ego_theta, ego_length, ego_width);

    // correct the inconsistency of reference point and center point
    // TODO(all): move the logic before constructing the ego_box
    double shift_distance = ego_length / 2.0 - ego_back_edge_to_center;
    Vec2d shift_vec(shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta));
    ego_box.Shift(shift_vec);

    std::vector<Box2d> obstacle_boxes;
    for (const auto obstacle : obstacles) {
      auto obtacle_point = obstacle->GetPointAtTime(relative_time);
      Box2d obstacle_box = obstacle->GetBoundingBox(obtacle_point);
      if (ego_box.HasOverlap(obstacle_box)) {
        return true;
      }
    }
  }
  return false;
}

bool CollisionChecker::InCollision(
    const DiscretizedTrajectory& discretized_trajectory) {
  CHECK_LE(discretized_trajectory.NumOfPoints(),
           predicted_bounding_rectangles_.size());
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  for (size_t i = 0; i < discretized_trajectory.NumOfPoints(); ++i) {
    const auto& trajectory_point =
        discretized_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);

    for (const auto& obstacle_box : predicted_bounding_rectangles_[i]) {
      if (ego_box.HasOverlap(obstacle_box)) {
        return true;
      }
    }
  }
  return false;
}

void CollisionChecker::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line) {
  CHECK(predicted_bounding_rectangles_.empty());

  // If the ego vehicle is in lane,
  // then, ignore all obstacles from the same lane.
  bool ego_vehicle_in_lane = IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d);
  std::vector<const Obstacle*> obstacles_considered;
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (ego_vehicle_in_lane &&
        (IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s,
                                    discretized_reference_line) ||
         !ptr_path_time_graph_->IsObstacleInGraph(obstacle->Id()))) {
      continue;
    }

    obstacles_considered.push_back(obstacle);
  }

  double relative_time = 0.0;
  while (relative_time < FLAGS_trajectory_time_length) {
    std::vector<Box2d> predicted_env;
    for (const Obstacle* obstacle : obstacles_considered) {
      // If an obstacle has no trajectory, it is considered as static.
      // Obstacle::GetPointAtTime has handled this case.
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      box.LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer);
      box.LateralExtend(2.0 * FLAGS_lat_collision_buffer);
      predicted_env.push_back(std::move(box));
    }
    predicted_bounding_rectangles_.push_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

bool CollisionChecker::IsEgoVehicleInLane(const double ego_vehicle_s,
                                          const double ego_vehicle_d) {
  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      ego_vehicle_s, &left_width, &right_width);
  return ego_vehicle_d < left_width && ego_vehicle_d > -right_width;
}

bool CollisionChecker::IsObstacleBehindEgoVehicle(
    const Obstacle* obstacle, const double ego_vehicle_s,
    const std::vector<PathPoint>& discretized_reference_line) {
  double half_lane_width = FLAGS_default_reference_line_width * 0.5;
  TrajectoryPoint point = obstacle->GetPointAtTime(0.0);
  auto obstacle_reference_line_position = PathMatcher::GetPathFrenetCoordinate(
      discretized_reference_line, point.path_point().x(),
      point.path_point().y());

  if (obstacle_reference_line_position.first < ego_vehicle_s &&
      std::fabs(obstacle_reference_line_position.second) < half_lane_width) {
    ADEBUG << "Ignore obstacle [" << obstacle->Id() << "]";
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
