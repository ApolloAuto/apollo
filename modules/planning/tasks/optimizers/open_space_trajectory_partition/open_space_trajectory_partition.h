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

#include "modules/planning/tasks/optimizers/trajectory_optimizer.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"

namespace apollo {
namespace planning {
class OpenSpaceTrajectoryPartition : public TrajectoryOptimizer {
 public:
  explicit OpenSpaceTrajectoryPartition(const TaskConfig& config);

  ~OpenSpaceTrajectoryPartition() = default;

  void Restart();

 private:
  apollo::common::Status Process(
      DiscretizedTrajectory* const trajectory_data) override;

 private:
  void InterpolateTrajectory(DiscretizedTrajectory* const trajectory,
                             DiscretizedTrajectory* interpolated_trajectory);

  bool InsertGearShiftTrajectory(
      const bool& flag_change_to_next, const size_t& current_trajectory_index,
      const std::vector<apollo::canbus::Chassis::GearPosition>& gear_positions);

  void GenerateGearShiftTrajectory(
      const apollo::canbus::Chassis::GearPosition& gear_position);

  void SetTrajectoryPb(
      const apollo::planning_internal::Trajectories& trajectory_partitioned,
      const std::vector<apollo::canbus::Chassis::GearPosition>& gear_positions,
      const size_t& current_trajectory_index,
      const int& closest_trajectory_point_index);
};
}  // namespace planning
}  // namespace apollo
