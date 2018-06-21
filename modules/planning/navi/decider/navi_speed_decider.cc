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
}

NaviSpeedDecider::NaviSpeedDecider() : Task("NaviSpeedDecider") {
  // TODO(all): Add your other initialization.
}

Status NaviSpeedDecider::Execute(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret = MakeSpeedDecision(
      frame_->vehicle_state(),
      frame_->obstacles(),
      reference_line_info_->mutable_speed_data());
  RecordDebugInfo(reference_line_info->speed_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  return ret;
}

Status NaviSpeedDecider::MakeSpeedDecision(
    const VehicleState& vehicle_state,
    const std::vector<const Obstacle*>& obstacles,
    SpeedData* const speed_data) {
  DCHECK_NOTNULL(speed_data);

  auto obstacle_closest_lon = kObstacleMaxLon;
  bool has_obstacle_speed = false;
  double obstacle_speed = 0.0;

  auto vehicle_speed = vehicle_state.has_linear_velocity() ?
      vehicle_state.linear_velocity() : 0.0;
  auto vehicle_acceleration = vehicle_state.has_linear_acceleration() ?
      vehicle_state.linear_acceleration() : 0.0;

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
      has_obstacle_speed && (obstacle_speed < FLAGS_default_cruise_speed) ?
      obstacle_speed : FLAGS_default_cruise_speed;

  // create speed-points
  speed_data->Clear();
  // the first point
  speed_data->AppendSpeedPoint(0.0, 0.0, speed, vehicle_acceleration, 0.0);
  // the second point
  auto time = obstacle_closest_lon / speed;
  speed_data->AppendSpeedPoint(obstacle_closest_lon, time, speed, 0.0, 0.0);

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
