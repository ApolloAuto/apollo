/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_open_space/utils/open_space_fallback_util.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::math::Polygon2d;
using apollo::common::TrajectoryPoint;

  bool OpenSpaceFallbackUtil::IsCollisionFreeTrajectory(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<std::vector<Box2d>>&
        predicted_bounding_rectangles,
    const std::vector<Obstacle*>& static_obstacles, int* current_index,
    int* first_collision_index, bool& is_collision_with_static_obstacle,
    bool& is_collision_with_dynamic_obstacle, const double& collilsion_time_buffer) {
  // prediction time resolution: FLAGS_trajectory_time_resolution
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();
  auto trajectory_pb = trajectory_gear_pair.first;
  const int point_size = trajectory_pb.NumOfPoints();
  AINFO << "trajectory_pb.NumOfPoints: [" << point_size << "]";
  *current_index = trajectory_pb.QueryLowerBoundPoint(0.0);

  for (int i = *current_index; i < point_size; ++i) {
    const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(i);
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);

    // is collision with static obstacles
    Polygon2d ego_polygon = Polygon2d(ego_box);
    for (const Obstacle* obstacle : static_obstacles) {
      Polygon2d obstacle_polygon = obstacle->PerceptionPolygon();
      if (ego_polygon.HasOverlap(obstacle_polygon)) {
        AINFO << "HasOverlapWithStaticObstacle" << obstacle->Id()
               << "first_collision_index_of_static_obstacle: [" << i << "]";
        *first_collision_index = i;
        is_collision_with_static_obstacle = true;
        return false;
      }
    }

    // is collision with dynamic obstacles
    int predicted_time_horizon = predicted_bounding_rectangles.size();
    for (int j = 0; j < predicted_time_horizon; j++) {
      for (const auto& obstacle_box : predicted_bounding_rectangles[j]) {
        if (ego_box.HasOverlap(obstacle_box)) {
          ADEBUG << "HasOverlap(obstacle_box) [" << i << "]";
          // remove points in previous trajectory
          if (std::abs(trajectory_point.relative_time() -
                       static_cast<double>(j) *
                           FLAGS_trajectory_time_resolution) <
                  collilsion_time_buffer &&
              trajectory_point.relative_time() > 0.0) {
            ADEBUG << "first_collision_index: [" << i << "]";
            *first_collision_index = i;
            is_collision_with_dynamic_obstacle = true;
            return false;
          }
        }
      }
    }
  }

  return true;
}

void OpenSpaceFallbackUtil::BuildPredictedEnvironment(
    const Vec2d ego_pos,
    const std::vector<const Obstacle*>& obstacles,
    std::vector<std::vector<common::math::Box2d>>&
        predicted_bounding_rectangles,
    const double& time_period,
    const double& build_range) {
  predicted_bounding_rectangles.clear();
  double relative_time = 0.0;
  while (relative_time < time_period) {
    std::vector<Box2d> predicted_env;
    std::vector<Obstacle*> static_obstacles_env;
    for (const Obstacle* obstacle : obstacles) {
      if (!obstacle->IsVirtual() && !obstacle->IsStatic()) {
        TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
        if (ego_pos.DistanceTo(Vec2d(point.path_point().x(), point.path_point().y())) > build_range) {
          continue;
        }
        Box2d box = obstacle->GetBoundingBox(point);
        predicted_env.push_back(std::move(box));
      }
    }
    predicted_bounding_rectangles.emplace_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

void OpenSpaceFallbackUtil::BuildStaticObstacleEnvironment(
    const Vec2d ego_pos,
    const std::vector<const Obstacle*>& obstacles,
    std::vector<Obstacle*>& static_obstacles,
    const double& build_range) {
  static_obstacles.clear();
  for (const Obstacle* obstacle : obstacles) {
    if (!obstacle->IsVirtual() && obstacle->IsStatic() &&
        obstacle->Perception().width() > 0.2 &&
        obstacle->Perception().length() > 0.2 &&
        obstacle->PerceptionPolygon().DistanceTo(ego_pos) <= build_range) {
      static_obstacles.push_back(const_cast<Obstacle*>(obstacle));
    }
  }
}

} // apollo
} // planning

