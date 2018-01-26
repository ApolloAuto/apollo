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

#include "modules/planning/lattice/trajectory_generator/backup_trajectory.h"
#include "modules/planning/lattice/trajectory_generator/trajectory_combiner.h"
namespace apollo {
namespace planning {

using apollo::common::PathPoint;

BackupTrajectory::BackupTrajectory(const std::array<double, 3>& init_s,
                                   const std::array<double, 3>& init_d,
                                   const double init_relative_time)
    : init_s_(init_s),
      init_d_(init_d),
      init_relative_time_(init_relative_time) {
  SetLonTrajectories();
  SetLatTrajectories();
}

void BackupTrajectory::SetLonTrajectories() {
  lon_trajectories_.clear();
  std::vector<double> accelerations{-1.0, -2.0, -3.0, -4.0};
  for (double acc : accelerations) {
    lon_trajectories_.emplace_back(std::unique_ptr<Curve1d>(
        new ConstantDecelerationTrajectory1d(init_s_[0], init_s_[1], acc)));
  }
}

void BackupTrajectory::SetLatTrajectories() {
  lat_trajectories_.clear();
  lat_trajectories_.emplace_back(std::unique_ptr<Curve1d>(
      new ConstantDecelerationTrajectory1d(init_d_[0], 0.0, 0.0)));
}

DiscretizedTrajectory BackupTrajectory::GenerateTrajectory(
    const std::vector<PathPoint>& discretized_ref_points) {
  return TrajectoryCombiner::Combine(
      discretized_ref_points, *lon_trajectories_.front(),
      *lat_trajectories_.front(), init_relative_time_);
}

}  // namespace planning
}  // namespace apollo
