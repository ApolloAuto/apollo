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

OpenSpaceFallbackDecider::OpenSpaceFallbackDecider(const TaskConfig& config)
    : Decider(config) {}

bool OpenSpaceFallbackDecider::QuardraticFormulaLowerSolution(const double a,
                                                              const double b,
                                                              const double c,
                                                              double* sol) {
  // quardratic formula: ax^2 + bx + c = 0, return lower solution

  // TODO(QiL): use const from common::math
  const double epis = 1e-6;
  *sol = 0.0;
  if (std::abs(a) < epis) {
    return false;
  }

  double tmp = b * b - 4 * a * c;
  if (tmp < epis) {
    return false;
  }

  *sol = (-b + std::sqrt(tmp)) / (2.0 * a);
  return true;
}

Status OpenSpaceFallbackDecider::Process(Frame* frame) {
  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles;
  size_t first_collision_idx = 0;
  size_t current_idx = 0;

  BuildPredictedEnvironment(frame->obstacles(), predicted_bounding_rectangles);
  ADEBUG << "Numbers of obstsacles are: " << frame->obstacles().size();
  ADEBUG << "Numbers of predicted bounding rectangles are: "
         << predicted_bounding_rectangles[0].size()
         << " and : " << predicted_bounding_rectangles.size();
  if (!IsCollisionFreeTrajectory(
          frame->open_space_info().chosen_paritioned_trajectory(),
          predicted_bounding_rectangles, &current_idx, &first_collision_idx)) {
    // change gflag
    frame_->mutable_open_space_info()->set_fallback_flag(true);

    // generate fallback trajectory base on current partition trajectory
    // vehicle speed is decreased to zero inside safety distance
    *(frame_->mutable_open_space_info()->mutable_fallback_trajectory()) =
        frame->open_space_info().chosen_paritioned_trajectory();
    auto* ptr_fallback_trajectory_pair =
        frame_->mutable_open_space_info()->mutable_fallback_trajectory();

    // Where is this fallback trajectory originally from
    const auto collision_point =
        ptr_fallback_trajectory_pair->first[first_collision_idx];
    auto fallback_start_point =
        ptr_fallback_trajectory_pair->first[current_idx];
    double relative_collision_distance = collision_point.path_point().s() -
                                         fallback_start_point.path_point().s();

    // TODO(QiL): move this to configs
    const double collision_buff_distance = 0.5;
    if (relative_collision_distance > collision_buff_distance) {
      relative_collision_distance -= collision_buff_distance;
    }
    // the accelerate = -v0^2 / (2*s), where s is slowing down distance
    const size_t temp_horizon =
        frame_->open_space_info().fallback_trajectory().first.NumOfPoints();
    const double accelerate = -fallback_start_point.v() *
                              fallback_start_point.v() /
                              (2.0 * (relative_collision_distance + 1e-6));
    const double v0 = fallback_start_point.v();

    for (size_t i = current_idx; i < temp_horizon; ++i) {
      double temp_relative_time = 0.0;
      double temp_v = 0.0;
      double b = 2.0 * v0;
      double c = -2.0 * ptr_fallback_trajectory_pair->first[i].path_point().s();
      if (QuardraticFormulaLowerSolution(accelerate, b, c,
                                         &temp_relative_time) &&
          ptr_fallback_trajectory_pair->first[i].path_point().s() <
              relative_collision_distance) {
        temp_v = v0 + accelerate * temp_relative_time;
        ptr_fallback_trajectory_pair->first[i].set_v(temp_v);
        ptr_fallback_trajectory_pair->first[i].set_a(accelerate);
        ptr_fallback_trajectory_pair->first[i].set_relative_time(
            temp_relative_time);
      } else {
        temp_relative_time = 0.04 + fallback_start_point.relative_time();
        ptr_fallback_trajectory_pair->first[i].mutable_path_point()->CopyFrom(
            fallback_start_point.path_point());
        ptr_fallback_trajectory_pair->first[i].set_v(0.0);
        ptr_fallback_trajectory_pair->first[i].set_a(0.0);
        ptr_fallback_trajectory_pair->first[i].set_relative_time(
            temp_relative_time);
      }
      fallback_start_point = ptr_fallback_trajectory_pair->first[i];
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
    size_t* current_idx, size_t* first_collision_idx) {
  // prediction time resolution: FLAGS_trajectory_time_resolution
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();
  auto trajectory_pb = trajectory_gear_pair.first;
  const size_t point_size = trajectory_pb.NumOfPoints();
  *current_idx = trajectory_pb.QueryLowerBoundPoint(0.0);

  for (size_t i = *current_idx; i < point_size; ++i) {
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
          if (std::abs(trajectory_point.relative_time() -
                       static_cast<double>(j) *
                           FLAGS_trajectory_time_resolution) <
              FLAGS_trajectory_time_resolution) {
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
