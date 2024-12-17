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

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/control_msgs/control_interactive_msg.pb.h"

#include "modules/planning/planning_base/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

namespace apollo {
namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  static void TransformLastPublishedTrajectory(
      const double x_diff, const double y_diff, const double theta_diff,
      PublishableTrajectory* prev_trajectory);

  static std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(
      const canbus::Chassis& vehicle_chassis,
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time, const size_t preserved_points_num,
      const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,
      std::string* replan_reason,
      const control::ControlInteractiveMsg& control_interactive_msg);

  static std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const double planning_cycle_time,
      const common::VehicleState& vehicle_state);

  static bool need_replan_by_necessary_check(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const PublishableTrajectory* prev_trajectory, std::string* replan_reason,
      size_t* time_matched_index);

  static bool need_replan_by_control_interactive(
      const double current_timestamp, std::string* replan_reason,
      const control::ControlInteractiveMsg& control_interactive_msg);

  static std::vector<common::TrajectoryPoint>
  ComputeControlInteractiveStitchingTrajectory(
      const double planning_cycle_time,
      const common::VehicleState& vehicle_state,
      const common::TrajectoryPoint& time_match_point,
      const control::ControlInteractiveMsg& control_interactive_msg);

 private:
  static std::pair<double, double> ComputePositionProjection(
      const double x, const double y,
      const common::TrajectoryPoint& matched_trajectory_point);

  static common::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      const double planning_cycle_time,
      const common::VehicleState& vehicle_state);
};

}  // namespace planning
}  // namespace apollo
