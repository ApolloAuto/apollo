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

#include <utility>

#include "modules/planning/lattice/util/collision_checker.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/common/log.h"

namespace apollo {
namespace planning {

CollisionChecker::CollisionChecker(
    const std::vector<const Obstacle*>& obstacles) {
  BuildPredictedEnv(obstacles);
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
        ego_length * (1.0 + adc_collision_buffer),
        ego_width * (1.0 + adc_collision_buffer));
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
    const std::vector<const Obstacle*>& obstacles) {
  CHECK(predicted_envs_.empty());

  double relative_time = 0.0;
  while (relative_time < planned_trajectory_time) {
    std::vector<common::math::Box2d> predicted_env;
    for (const Obstacle* obstacle : obstacles) {
      // TODO(yajia):
      // temp. fix, figure out why some obstacle's predicted trajectory
      // has zero points.
      if (obstacle->Trajectory().trajectory_point_size() == 0) {
        continue;
      }
      common::TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      common::math::Box2d box = obstacle->GetBoundingBox(point);
      predicted_env.push_back(std::move(box));
    }
    predicted_envs_.push_back(std::move(predicted_env));
    relative_time += trajectory_time_resolution;
  }
}

}  // namespace planning
}  // namespace apollo
