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

#ifndef MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATOR_TRAJECTORY_EVALUATOR_H_
#define MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATOR_TRAJECTORY_EVALUATOR_H_

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "modules/planning/lattice/behavior_decider/path_time_neighborhood.h"
#include "modules/planning/math/curve1d/curve1d.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class TrajectoryEvaluator {
  // normal use
  typedef std::pair<
      std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>, double>
      PairCost;

  // auto tuning
  typedef std::pair<
      std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>,
      std::pair<std::vector<double>, double>> PairCostWithComponents;

 public:
  explicit TrajectoryEvaluator(
      const PlanningTarget& planning_target,
      const std::vector<std::shared_ptr<Curve1d>>& lon_trajectories,
      const std::vector<std::shared_ptr<Curve1d>>& lat_trajectories,
      bool is_auto_tuning,
      std::shared_ptr<PathTimeNeighborhood> pathtime_neighborhood);

  virtual ~TrajectoryEvaluator() = default;

  bool has_more_trajectory_pairs() const;

  std::size_t num_of_trajectory_pairs() const;

  std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>
  next_top_trajectory_pair();

  double top_trajectory_pair_cost() const;

  std::vector<double> top_trajectory_pair_component_cost() const;

  std::vector<double> evaluate_per_lonlat_trajectory(
      const PlanningTarget& planning_target,
      const std::vector<apollo::common::SpeedPoint> st_points,
      const std::vector<apollo::common::FrenetFramePoint> sl_points);

 private:
  double Evaluate(
      const PlanningTarget& planning_target,
      const std::shared_ptr<Curve1d>& lon_trajectory,
      const std::shared_ptr<Curve1d>& lat_trajectory,
      std::vector<double>* cost_components) const;

  double LatOffsetCost(
      const std::shared_ptr<Curve1d>& lat_trajectory,
      const std::vector<double>& s_values) const;

  double LonComfortCost(
      const std::shared_ptr<Curve1d>& lon_trajectory) const;

  double LonCollisionCost(
      const std::shared_ptr<Curve1d>& lon_trajectory) const;

  double LonObjectiveCost(
      const std::shared_ptr<Curve1d>& lon_trajectory,
      const PlanningTarget& planning_target) const;

  struct CostComparator
      : public std::binary_function<const PairCost&, const PairCost&, bool> {
    bool operator()(const PairCost& left, const PairCost& right) const {
      return left.second > right.second;
    }
  };

  struct CostComponentComparator
      : public std::binary_function<const PairCostWithComponents&,
                                    const PairCostWithComponents&, bool> {
    bool operator()(const PairCostWithComponents& left,
                    const PairCostWithComponents& right) const {
      return left.second.second > right.second.second;
    }
  };

  std::priority_queue<PairCost, std::vector<PairCost>, CostComparator>
      cost_queue_;

  std::priority_queue<PairCostWithComponents,
                      std::vector<PairCostWithComponents>,
                      CostComponentComparator> cost_queue_with_components_;

  bool is_auto_tuning_ = false;

  std::shared_ptr<PathTimeNeighborhood> pathtime_neighborhood_;
};

}  // namespace planning
}  // namespace apollo

#endif  // PLANNING_LATTICE_TRAJECTORY_GENERATOR_TRAJECTORY_EVALUATOR_H_
