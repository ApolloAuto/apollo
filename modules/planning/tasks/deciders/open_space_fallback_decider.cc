/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/open_space_fallback_decider.h"

namespace apollo {
namespace planning {
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::planning::TrajGearPair;

OpenSpaceFallbackDecider::OpenSpaceFallbackDecider(const TaskConfig& config)
    : Decider(config) {}

Status OpenSpaceFallbackDecider::Process(Frame* frame) {
  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles;
  double collision_distance;

  BuildPredictedEnvironment(frame->obstacles(), predicted_bounding_rectangles);

  if (!IsCollisionFreeTrajectory(
          frame->open_space_info().chosen_paritioned_trajectory(),
          predicted_bounding_rectangles, &collision_distance)) {
    // change gflag
    frame_->mutable_open_space_info()->set_fallback_flag(true);

    // generate fallback trajectory base on current partition trajectory
    // vehicle speed is decreased to zero inside safety distance
    *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
        frame->open_space_info().chosen_paritioned_trajectory();
    auto fallback_trajectory_vec =
        frame_->mutable_open_space_info()->mutable_fallback_trajectory()->first;

    double stop_distance = std::max(0.0,
        collision_distance - config_.open_space_fallback_decider_config().
            open_space_fall_back_stop_safety_gap());
    if (stop_distance > 0.0) {
      // the accelerate = v0^2 / (2*s), where s is slowing down distance
      double accelerate =
          (frame_->vehicle_state().linear_velocity() *
           frame_->vehicle_state().linear_velocity()) /
           2.0 / stop_distance;
      double current_v = frame_->vehicle_state().linear_velocity();
      size_t temp_horizon =
          frame_->open_space_info().fallback_trajectory().first.NumOfPoints();
      for (size_t i = 0; i < temp_horizon; ++i) {
        double next_v = std::max(0.0, current_v -
            accelerate * frame_->open_space_info().
                fallback_trajectory().first.
                TrajectoryPointAt(i).relative_time());
        fallback_trajectory_vec[i].set_v(next_v);
        fallback_trajectory_vec[i].set_a(-accelerate);
        current_v = next_v;
      }
    } else {
      // if the stop distance is not enough, stop at current location
      size_t temp_horizon =
          frame_->open_space_info().fallback_trajectory().first.NumOfPoints();
      for (size_t i = 0; i < temp_horizon; ++i) {
        fallback_trajectory_vec[i].set_v(0.0);
      }
    }
  } else {
    frame_->mutable_open_space_info()->set_fallback_flag(false);
  }

  return Status::OK();
}

void OpenSpaceFallbackDecider::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles,
    std::vector<std::vector<common::math::Box2d>>&
        predicted_bounding_rectangles) {
  predicted_bounding_rectangles.clear();
  double relative_time = 0.0;
  while (relative_time < config_.open_space_fallback_decider_config()
                             .open_space_prediction_time_period()) {
    std::vector<Box2d> predicted_env;
    for (const Obstacle* obstacle : obstacles) {
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      predicted_env.push_back(std::move(box));
    }
    predicted_bounding_rectangles.emplace_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

bool OpenSpaceFallbackDecider::IsCollisionFreeTrajectory(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<std::vector<common::math::Box2d>>&
        predicted_bounding_rectangles,
    double* collision_distance) {
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();
  auto trajectory_pb = trajectory_gear_pair.first;
  const size_t point_size = trajectory_pb.NumOfPoints();

  for (size_t i = 0; i < point_size; ++i) {
    const auto& trajectory_point = trajectory_pb.TrajectoryPointAt(i);
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(),
         trajectory_point.path_point().y()},
         ego_theta, ego_length, ego_width);
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);
    size_t predicted_time_horizon = predicted_bounding_rectangles.size();
    for (size_t j = 0; j < predicted_time_horizon; j++) {
      for (const auto& obstacle_box : predicted_bounding_rectangles[j]) {
        if (ego_box.HasOverlap(obstacle_box)) {
          const auto& vehicle_state = frame_->vehicle_state();
          Vec2d vehicle_vec({vehicle_state.x(), vehicle_state.y()});
          if (obstacle_box.DistanceTo(vehicle_vec) <
              config_.open_space_fallback_decider_config()
                  .open_space_fall_back_collision_distance()) {
            *collision_distance = obstacle_box.DistanceTo(vehicle_vec);
            return false;
          }
        }
      }
    }
  }

  return true;
}
}  // namespace planning
}  // namespace apollo
