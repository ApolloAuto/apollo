/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @brief This file provides the implementation of the class "NaviSpeedDecider".
 */

#include "modules/planning/navi/decider/navi_speed_decider.h"

#include <algorithm>
#include <cmath>

#include "glog/logging.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleState;

namespace {
// max distance of obstacle
constexpr double kObstacleMaxLon = 999.0;
}  // namespace

NaviSpeedDecider::NaviSpeedDecider() : Task("NaviSpeedDecider") {}

bool NaviSpeedDecider::Init(const PlanningConfig& config) {
  CHECK(config.has_navi_planner_config());
  CHECK(config.navi_planner_config().has_navi_speed_decider_config());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_preferred_accel());
  CHECK(config.navi_planner_config()
            .navi_speed_decider_config()
            .has_preferred_decel());
  CHECK(
      config.navi_planner_config().navi_speed_decider_config().has_max_accel());
  CHECK(
      config.navi_planner_config().navi_speed_decider_config().has_max_decel());

  config_ = config;
  UpdateAccelSettings(
      config_.navi_planner_config()
          .navi_speed_decider_config()
          .preferred_accel(),
      config_.navi_planner_config()
          .navi_speed_decider_config()
          .preferred_decel(),
      config_.navi_planner_config().navi_speed_decider_config().max_accel(),
      config_.navi_planner_config().navi_speed_decider_config().max_decel());

  return true;
}

Status NaviSpeedDecider::Execute(Frame* frame,
                                 ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret = MakeSpeedDecision(frame_->vehicle_state(), frame_->obstacles(),
                               reference_line_info_->mutable_speed_data());
  RecordDebugInfo(reference_line_info->speed_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  return ret;
}

void NaviSpeedDecider::UpdateAccelSettings(double preferred_accel,
                                           double preferred_decel,
                                           double max_accel, double max_decel) {
  max_accel_ = std::abs(max_accel);
  max_decel_ = std::abs(max_decel);
  preferred_accel_ = std::min(std::abs(preferred_accel), max_accel_);
  preferred_decel_ = std::min(std::abs(preferred_decel), max_decel_);
}

Status NaviSpeedDecider::MakeSpeedDecision(
    const VehicleState& vehicle_state,
    const std::vector<const Obstacle*>& obstacles,
    SpeedData* const speed_data) {
  CHECK_NOTNULL(speed_data);

  auto obstacle_closest_lon = kObstacleMaxLon;
  bool has_obstacle_speed = false;
  double obstacle_speed = 0.0;

  auto vehicle_speed = vehicle_state.has_linear_velocity()
                           ? vehicle_state.linear_velocity()
                           : 0.0;

  const auto& vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  auto front_edge_to_center =
      vehicle_config.vehicle_param().front_edge_to_center();

  for (const auto* obstacle : obstacles) {
    // using FLU
    const auto obstacle_aa_box = obstacle->PerceptionBoundingBox().GetAABox();
    // TODO(all): if distance < 0 ?
    auto distance = obstacle_aa_box.min_x() - front_edge_to_center;
    // get the obstacle with minimum distance
    if (distance < obstacle_closest_lon) {
      obstacle_closest_lon = distance;
      has_obstacle_speed = true;

      double rel_speed = 0.0;
      if (obstacle->Perception().has_velocity() &&
          obstacle->Perception().velocity().has_x())
        rel_speed = obstacle->Perception().velocity().x();
      // TODO(all): if obstacle_speed < 0 ?
      obstacle_speed = rel_speed + vehicle_speed;
    }
  }

  // decide speed
  auto speed =
      has_obstacle_speed && (obstacle_speed < FLAGS_default_cruise_speed)
          ? obstacle_speed
          : FLAGS_default_cruise_speed;

  // decide acceleration
  double accel;
  double accel_time;
  if (vehicle_speed < speed) {
    accel = preferred_accel_;
    accel_time = (speed - vehicle_speed) / preferred_accel_;
  } else if (vehicle_speed > speed) {
    accel = -1 * preferred_decel_;
    accel_time = (vehicle_speed - speed) / preferred_decel_;
  } else {
    accel = 0.0;
    accel_time = 0.0;
  }
  auto accel_distance = 0.5 * (speed + vehicle_speed) * accel_time;

  // create speed-points
  speed_data->Clear();

  // The actual speed of the vehicle is almost the same as the cruising speed.
  if (!(accel_distance > 0.0)) {
    // the first point
    speed_data->AppendSpeedPoint(0.0, 0.0, vehicle_speed, accel, 0.0);
    // the second point
    auto total_time = obstacle_closest_lon / speed;
    speed_data->AppendSpeedPoint(obstacle_closest_lon, total_time, speed, 0.0,
                                 0.0);

    return Status::OK();
  }

  if (obstacle_closest_lon > accel_distance) {
    // the first point
    speed_data->AppendSpeedPoint(0.0, 0.0, vehicle_speed, accel, 0.0);
    // the second point
    speed_data->AppendSpeedPoint(accel_distance, accel_time, speed, 0.0, 0.0);
    // the third point
    auto total_time =
        (obstacle_closest_lon - accel_distance) / speed + accel_time;
    speed_data->AppendSpeedPoint(obstacle_closest_lon, total_time, speed, 0.0,
                                 0.0);
  } else {
    // TODO(all): need do more.
    // the first point
    speed_data->AppendSpeedPoint(0.0, 0.0, vehicle_speed, accel, 0.0);
    // the second point
    auto total_time = obstacle_closest_lon / speed;
    speed_data->AppendSpeedPoint(obstacle_closest_lon, total_time, speed, 0.0,
                                 0.0);
  }

  return Status::OK();
}

void NaviSpeedDecider::RecordDebugInfo(const SpeedData& speed_data) {
  auto* debug = reference_line_info_->mutable_debug();
  auto ptr_speed_plan = debug->mutable_planning_data()->add_speed_plan();
  ptr_speed_plan->set_name(Name());
  ptr_speed_plan->mutable_speed_point()->CopyFrom(
      {speed_data.speed_vector().begin(), speed_data.speed_vector().end()});
}

}  // namespace planning
}  // namespace apollo
