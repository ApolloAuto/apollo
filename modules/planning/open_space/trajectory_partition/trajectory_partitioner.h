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
 **/

#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/common/trajectory/trajectory_stitcher.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class TrajectoryPartitioner {
 public:
  TrajectoryPartitioner();

  void Restart();

  apollo::common::Status TrajectoryPartition(
      const std::unique_ptr<PublishableTrajectory>& last_publishable_trajectory,
      const Frame* frame, ADCTrajectory* const ptr_trajectory_pb);

 private:
  void InterpolateTrajectory(
      const std::unique_ptr<PublishableTrajectory>& last_publishable_trajectory,
      std::vector<common::TrajectoryPoint>* zigzag_trajectory);
  bool InsertGearShiftTrajectory(
      const bool& flag_change_to_next, const size_t& current_trajectory_index,
      const std::vector<apollo::canbus::Chassis::GearPosition>& gear_positions,
      const Frame* frame, ADCTrajectory* const ptr_trajectory_pb);
  void GenerateGearShiftTrajectory(
      const apollo::canbus::Chassis::GearPosition& gear_position,
      const Frame* frame, ADCTrajectory* trajectory_pb);
  void SetTrajectoryPb(
      const apollo::planning_internal::Trajectories& trajectory_partitioned,
      const std::vector<apollo::canbus::Chassis::GearPosition>& gear_positions,
      const size_t& current_trajectory_index,
      const int& closest_trajectory_point_index,
      ADCTrajectory* const ptr_trajectory_pb);

 private:
  PlannerOpenSpaceConfig planner_open_space_config_;
  bool gear_shift_period_finished_ = true;
  bool gear_shift_period_started_ = true;
  double gear_shift_period_time_ = 0.0;
  double gear_shift_start_time_ = 0.0;
  apollo::canbus::Chassis::GearPosition gear_shift_position_ =
      canbus::Chassis::GEAR_DRIVE;
  double gear_shift_max_t_ = 0.0;
  double gear_shift_unit_t_ = 0.0;
  size_t interpolated_pieces_num_ = 0;
  size_t initial_gear_check_horizon_ = 0;
  double heading_searching_range_ = 0.0;
  double gear_shift_period_duration_ = 0.0;
};
}  // namespace planning
}  // namespace apollo
