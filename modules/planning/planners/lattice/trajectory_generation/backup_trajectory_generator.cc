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

#include "modules/planning/planners/lattice/trajectory_generation/backup_trajectory_generator.h"

#include "modules/planning/planners/lattice/trajectory_generation/trajectory_combiner.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using State = std::array<double, 3>;

BackupTrajectoryGenerator::BackupTrajectoryGenerator(
    const State& init_s, const State& init_d, const double init_relative_time,
    const std::shared_ptr<CollisionChecker>& ptr_collision_checker,
    const Trajectory1dGenerator* trajectory1d_generator)
    : init_relative_time_(init_relative_time),
      ptr_collision_checker_(ptr_collision_checker),
      ptr_trajectory1d_generator_(trajectory1d_generator) {
  GenerateTrajectory1dPairs(init_s, init_d);
}

void BackupTrajectoryGenerator::GenerateTrajectory1dPairs(const State& init_s,
                                                          const State& init_d) {
  std::vector<std::shared_ptr<Curve1d>> lon_trajectories;
  std::array<double, 5> dds_condidates = {-0.1, -1.0, -2.0, -3.0, -4.0};
  for (const auto dds : dds_condidates) {
    lon_trajectories.emplace_back(
        new ConstantDecelerationTrajectory1d(init_s[0], init_s[1], dds));
  }

  std::vector<std::shared_ptr<Curve1d>> lat_trajectories;
  ptr_trajectory1d_generator_->GenerateLateralTrajectoryBundle(
      &lat_trajectories);

  for (auto& lon : lon_trajectories) {
    for (auto& lat : lat_trajectories) {
      trajectory_pair_pqueue_.emplace(lon, lat);
    }
  }
}

DiscretizedTrajectory BackupTrajectoryGenerator::GenerateTrajectory(
    const std::vector<PathPoint>& discretized_ref_points) {
  while (trajectory_pair_pqueue_.size() > 1) {
    auto top_pair = trajectory_pair_pqueue_.top();
    trajectory_pair_pqueue_.pop();
    DiscretizedTrajectory trajectory =
        TrajectoryCombiner::Combine(discretized_ref_points, *top_pair.first,
                                    *top_pair.second, init_relative_time_);
    if (!ptr_collision_checker_->InCollision(trajectory)) {
      return trajectory;
    }
  }
  auto top_pair = trajectory_pair_pqueue_.top();
  return TrajectoryCombiner::Combine(discretized_ref_points, *top_pair.first,
                                     *top_pair.second, init_relative_time_);
}

}  // namespace planning
}  // namespace apollo
