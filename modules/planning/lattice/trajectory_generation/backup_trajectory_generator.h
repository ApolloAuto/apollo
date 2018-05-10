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

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_BACKUP_TRAJECTORY_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_BACKUP_TRAJECTORY_H_

#include <array>
#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "modules/planning/lattice/trajectory_generation/trajectory1d_generator.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/lattice/trajectory1d/constant_deceleration_trajectory1d.h"
#include "modules/planning/math/curve1d/curve1d.h"

namespace apollo {
namespace planning {

class BackupTrajectoryGenerator {
 public:
  typedef std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>
      Trajectory1dPair;
  typedef std::pair<Trajectory1dPair, double> PairCost;

  BackupTrajectoryGenerator(
      const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
      const double init_relative_time,
      const Trajectory1dGenerator* trajectory1d_generator);

  DiscretizedTrajectory GenerateTrajectory(
      const std::vector<common::PathPoint>& discretized_ref_points);

 private:
  void GenerateTrajectory1dPairs(const std::array<double, 3>& init_s,
                                 const std::array<double, 3>& init_d);

  double init_relative_time_;

  const Trajectory1dGenerator* ptr_trajectory1d_generator_;

  struct CostComparator
      : public std::binary_function<const Trajectory1dPair&,
                                    const Trajectory1dPair&, bool> {
    bool operator()(const Trajectory1dPair& left,
                    const Trajectory1dPair& right) const {
      auto lon_left = left.first;
      auto lon_right = right.first;
      auto s_dot_left = lon_left->Evaluate(1, FLAGS_trajectory_time_length);
      auto s_dot_right = lon_right->Evaluate(1, FLAGS_trajectory_time_length);
      if (s_dot_left < s_dot_right) {
        return true;
      }
      return false;
    }
  };

  std::priority_queue<Trajectory1dPair, std::vector<Trajectory1dPair>,
                      CostComparator>
      trajectory_pair_pqueue_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_BACKUP_TRAJECTORY_H_
