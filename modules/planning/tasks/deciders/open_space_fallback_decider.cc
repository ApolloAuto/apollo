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
  double obstacle_to_vehicle_distance = 0.0;
  size_t first_collision_idx = 0;

  BuildPredictedEnvironment(frame->obstacles(), predicted_bounding_rectangles);
  ADEBUG << "Numbers of obstsacles are: " << frame->obstacles().size();
  ADEBUG << "Numbers of predicted bounding rectangles are: "
         << predicted_bounding_rectangles[0].size()
         << " and : " << predicted_bounding_rectangles.size();

  if (!IsCollisionFreeTrajectory(
          frame->open_space_info().chosen_paritioned_trajectory(),
          predicted_bounding_rectangles, &obstacle_to_vehicle_distance,
          &first_collision_idx)) {
    // change gflag
    frame_->mutable_open_space_info()->set_fallback_flag(true);

    // generate fallback trajectory base on current partition trajectory
    // vehicle speed is decreased to zero inside safety distance
    *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
        frame->open_space_info().chosen_paritioned_trajectory();
    auto ptr_fallback_trajectory_pair =
        frame_->mutable_open_space_info()->mutable_fallback_trajectory();

    const auto collision_point =
        ptr_fallback_trajectory_pair->first[first_collision_idx];
    auto previous_point = ptr_fallback_trajectory_pair->first[0];

    double relative_collision_point_x =
        collision_point.path_point().x() - previous_point.path_point().x();
    double relative_collision_point_y =
        collision_point.path_point().y() - previous_point.path_point().y();
    double relative_collision_distance =
        std::sqrt(relative_collision_point_x * relative_collision_point_x +
                  relative_collision_point_y * relative_collision_point_y);
    double stopping_distance =
        std::min(relative_collision_distance,
                 config_.open_space_fallback_decider_config()
                     .open_space_fall_back_stop_distance());

    if (stopping_distance > 0.0) {
      // the accelerate = -v0^2 / (2*s), where s is slowing down distance
      size_t temp_horizon =
          frame_->open_space_info().fallback_trajectory().first.NumOfPoints();

      const double accelerate =
          -previous_point.v() * previous_point.v() / (2.0 * stopping_distance);

      for (size_t i = 1; i < temp_horizon; ++i) {
        double temp_relative_time =
            ptr_fallback_trajectory_pair->first[i].relative_time() -
            previous_point.relative_time();
        double temp_v = previous_point.v() + accelerate * temp_relative_time;

        if (temp_v <= 0.0) {
          ptr_fallback_trajectory_pair->first[i].mutable_path_point()->set_x(
              previous_point.path_point().x());
          ptr_fallback_trajectory_pair->first[i].mutable_path_point()->set_y(
              previous_point.path_point().y());
          ptr_fallback_trajectory_pair->first[i].set_v(0.0);
          ptr_fallback_trajectory_pair->first[i].set_a(0.0);
        } else {
          ptr_fallback_trajectory_pair->first[i].set_v(temp_v);
          ptr_fallback_trajectory_pair->first[i].set_a(accelerate);
        }

        previous_point = ptr_fallback_trajectory_pair->first[i];
      }
    } else {
      // if the stop at first idx
      size_t temp_horizon =
          frame_->open_space_info().fallback_trajectory().first.NumOfPoints();
      const auto first_path_point =
          ptr_fallback_trajectory_pair->first[0].path_point();
      for (size_t i = 0; i < temp_horizon; ++i) {
        ptr_fallback_trajectory_pair->first[i].set_v(0.0);
        ptr_fallback_trajectory_pair->first[i].mutable_path_point()->set_x(
            first_path_point.x());
        ptr_fallback_trajectory_pair->first[i].mutable_path_point()->set_y(
            first_path_point.y());
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
      if (!obstacle->IsVirtual()) {
        TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
        Box2d box = obstacle->GetBoundingBox(point);
        predicted_env.push_back(std::move(box));
      }
    }
    predicted_bounding_rectangles.emplace_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

bool OpenSpaceFallbackDecider::IsCollisionFreeTrajectory(
    const TrajGearPair& trajectory_gear_pair,
    const std::vector<std::vector<common::math::Box2d>>&
        predicted_bounding_rectangles,
    double* obstacle_to_vehicle_distance, size_t* first_collision_idx) {
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
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
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
            *obstacle_to_vehicle_distance =
                obstacle_box.DistanceTo(vehicle_vec);
            *first_collision_idx = i;
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
