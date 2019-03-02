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

#pragma once

#include <utility>
#include <vector>

#include "modules/planning/tasks/optimizers/trajectory_optimizer.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {

typedef std::pair<DiscretizedTrajectory, canbus::Chassis::GearPosition>
    TrajGearPair;

class OpenSpaceTrajectoryPartition : public TrajectoryOptimizer {
 public:
  explicit OpenSpaceTrajectoryPartition(const TaskConfig& config);

  ~OpenSpaceTrajectoryPartition() = default;

  void Restart();

 private:
  common::Status Process() override;

  void InterpolateTrajectory(const DiscretizedTrajectory& trajectory,
                             DiscretizedTrajectory* interpolated_trajectory);

  bool InsertGearShiftTrajectory(
      const bool& flag_change_to_next, const size_t& current_trajectory_index,
      const std::vector<TrajGearPair>& paritioned_trajectories,
      TrajGearPair* gear_switch_idle_time_trajectory);

  void GenerateGearShiftTrajectory(
      const canbus::Chassis::GearPosition& gear_position,
      TrajGearPair* gear_switch_idle_time_trajectory);

  void AdjustRelativeTimeAndS(
      const std::vector<TrajGearPair>& paritioned_trajectories,
      const size_t& current_trajectory_index,
      const size_t& closest_trajectory_point_index,
      TrajGearPair* current_paritioned_trajectory);
};
}  // namespace planning
}  // namespace apollo
