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
 * @file trajectory_evaluator.h
 **/

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_TRAJECTORY_EVALUATOR_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_TRAJECTORY_EVALUATOR_H_

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "modules/planning/math/curve1d/curve1d.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class TrajectoryEvaluator {
  typedef std::pair<
      std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>, double> PairCost;

 public:
  explicit TrajectoryEvaluator(const PlanningTarget& planning_target,
      const std::vector<std::shared_ptr<Curve1d>>& lon_trajectories,
      const std::vector<std::shared_ptr<Curve1d>>& lat_trajectories);

  virtual ~TrajectoryEvaluator() = default;

  bool has_more_trajectory_pairs() const;

  std::size_t num_of_trajectory_pairs() const;

  std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>
  next_top_trajectory_pair();

  double top_trajectory_pair_cost() const;

 private:
  double evaluate(const PlanningTarget& planning_target,
      const std::shared_ptr<Curve1d>& lon_trajectory,
      const std::shared_ptr<Curve1d>& lat_trajectory) const;

  double compute_lat_trajectory_offset_cost(
      const std::shared_ptr<Curve1d>& lat_trajectory,
      const std::vector<double>& s_values) const;

  double compute_lon_trajectory_comfort_cost(
      const std::shared_ptr<Curve1d>& lon_trajectory) const;

  /**
   double compute_lon_trajectory_objective_cost(
   const std::shared_ptr<Curve1d>& lon_trajectory,
   const PlanningTarget& objective) const;
   **/

  struct CostComparator: public std::binary_function<const PairCost&,
      const PairCost&, bool> {
      bool operator()(const PairCost& left, const PairCost& right) const {
        return left.second > right.second;
      }
  };

  std::priority_queue<PairCost, std::vector<PairCost>, CostComparator> cost_queue_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY1D_GENERATOR_TRAJECTORY_EVALUATOR_H_
