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
 * @file collision_checker.cpp
 **/

#include "modules/planning/constraint_checker/collision_checker.h"

#include <array>
#include <cmath>
#include <utility>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/reference_line_frame_converter.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;

CollisionChecker::CollisionChecker(
    const std::vector<const Obstacle*>& obstacles,
    const std::array<double, 3>& adc_init_s,
    const std::array<double, 3>& adc_init_d,
    const std::vector<PathPoint>& discretized_reference_line) {
  BuildPredictedEnv(obstacles, adc_init_s, adc_init_d,
                    discretized_reference_line);
}

bool CollisionChecker::InCollision(
    const DiscretizedTrajectory& discretized_trajectory) {
  CHECK_LE(discretized_trajectory.NumOfPoints(), predicted_envs_.size());
  const auto& vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  std::size_t time_index = 0;
  for (const auto& trajectory_point :
       discretized_trajectory.trajectory_points()) {
    common::math::Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        trajectory_point.path_point().theta(),
        ego_length * (1.0 + FLAGS_collision_buffer_expansion_ratio),
        ego_width * (1.0 + FLAGS_collision_buffer_expansion_ratio));
    for (const auto& obstacle_box : predicted_envs_[time_index]) {
      if (ego_box.HasOverlap(obstacle_box)) {
        return true;
      }
    }
    ++time_index;
  }
  return false;
}

void CollisionChecker::BuildPredictedEnv(
    const std::vector<const Obstacle*>& obstacles,
    const std::array<double, 3>& adc_init_s,
    const std::array<double, 3>& adc_init_d,
    const std::vector<PathPoint>& discretized_reference_line) {
  CHECK(predicted_envs_.empty());

  bool ignore_obstacles_behind = IgnoreObstaclesBehind(adc_init_d);
  std::vector<const Obstacle*> obstacles_considered;
  for (const Obstacle* obstacle : obstacles) {
    if (ignore_obstacles_behind &&
        IsBehind(obstacle, adc_init_s, discretized_reference_line)) {
      continue;
    }
    obstacles_considered.push_back(obstacle);
  }

  double relative_time = 0.0;
  while (relative_time < FLAGS_trajectory_time_length) {
    std::vector<common::math::Box2d> predicted_env;
    for (const Obstacle* obstacle : obstacles_considered) {
      // If an obstacle has no trajectory, it is considered as static.
      // Obstacle::GetPointAtTime has handled this case.
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      predicted_env.push_back(std::move(box));
    }
    predicted_envs_.push_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

bool CollisionChecker::IgnoreObstaclesBehind(
    const std::array<double, 3>& adc_init_d) {
  return std::abs(adc_init_d[0]) < FLAGS_default_reference_line_width * 0.5;
}

bool CollisionChecker::IsBehind(
    const Obstacle* obstacle, const std::array<double, 3>& adc_init_s,
    const std::vector<PathPoint>& discretized_reference_line) {
  double half_lane_width = FLAGS_default_reference_line_width * 0.5;
  TrajectoryPoint point = obstacle->GetPointAtTime(0.0);
  std::pair<double, double> sl_point =
      ReferenceLineFrameConverter::CartesianToFrenet(discretized_reference_line,
                                                     point.path_point().x(),
                                                     point.path_point().y());
  if (sl_point.first < adc_init_s[0] &&
      std::abs(sl_point.second) < half_lane_width) {
    ADEBUG << "Ignore obstacle [" << obstacle->Id() << "]";
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
