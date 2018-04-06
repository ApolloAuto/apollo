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

#ifndef MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_
#define MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/reference_line/reference_line.h"

#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  /**
   * @brief Stitch to reference line based on location
   * Find the init location that helps the vehicle approach the reference line.
   * Only used in navigation mode.
   */
  static std::vector<common::TrajectoryPoint> CalculateInitPoint(
      const common::VehicleState& vehicle_state,
      const ReferenceLine& reference_line, bool* is_replan);

  static void TransformLastPublishedTrajectory(
      const double planning_cycle_time, PublishableTrajectory* prev_trajectory);

  static std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(
      const common::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory, bool* is_replan);

 private:
  static std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const common::VehicleState& vehicle_state);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_
