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

#include "modules/planning/lattice/trajectory1d_generator/trajectory_evaluator.h"

#include <cmath>
#include <limits>
#include <utility>
#include <functional>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/planning/lattice/util/lattice_constraint_checker.h"

namespace apollo {
namespace planning {

TrajectoryEvaluator::TrajectoryEvaluator(
    const PlanningTarget& objective,
    const std::vector<std::shared_ptr<Curve1d>>& lon_trajectories,
    const std::vector<std::shared_ptr<Curve1d>>& lat_trajectories) {
  for (std::size_t i = 0; i < lon_trajectories.size(); ++i) {
    if (!LatticeConstraintChecker::IsValidLongitudinalTrajectory(
            *lon_trajectories[i])) {
      continue;
    }
    for (std::size_t j = 0; j < lat_trajectories.size(); ++j) {
      if (!LatticeConstraintChecker::IsValidLateralTrajectory(
              *lat_trajectories[j], *lon_trajectories[i])) {
        continue;
      }
      double cost =
          evaluate(objective, lon_trajectories[i], lat_trajectories[j]);
      cost_queue_.push(
          PairCost(std::pair<std::size_t, std::size_t>(i, j), cost));
    }
  }
}

bool TrajectoryEvaluator::has_more_trajectory_pairs() const {
  return !cost_queue_.empty();
}

size_t TrajectoryEvaluator::num_of_trajectory_pairs() const {
  return cost_queue_.size();
}

std::pair<std::size_t, std::size_t>
TrajectoryEvaluator::next_top_trajectory_pair() {
  CHECK(has_more_trajectory_pairs() == true);
  auto top = cost_queue_.top();
  cost_queue_.pop();
  return top.first;
}

std::pair<std::size_t, std::size_t>
TrajectoryEvaluator::top_trajectory_pair_index() const {
  return cost_queue_.top().first;
}

double TrajectoryEvaluator::top_trajectory_pair_cost() const {
  return cost_queue_.top().second;
}

double TrajectoryEvaluator::evaluate(
    const PlanningTarget& objective,
    const std::shared_ptr<Curve1d>& lon_trajectory,
    const std::shared_ptr<Curve1d>& lat_trajectory) const {
  // currently consider three costs:
  // 1. the cost of jerk, currently only consider longitudinal jerk.
  // 2. the cost of time to achieve the objective
  // 3. the cost of failure to achieve the planning objective.

  // these numbers are for temporary use; needs to be tuned later.
  double weight_lon_jerk = 1.0;
  double weight_lon_objective = 3.0;
  double weight_lat_offset = 10.0;

  double t = 0.0;
  double t_max = lon_trajectory->param_length();
  double s_max = lon_trajectory->Evaluate(0, t_max);
  double v_end = lon_trajectory->Evaluate(1, t_max);

  std::vector<double> s_values;
  while (t < planned_trajectory_time) {
    double s = 0.0;
    if (t < t_max) {
      s = lon_trajectory->Evaluate(0, t);
    } else {
      // extend by constant speed movement
      s = s_max + v_end * (t - t_max);
    }
    s_values.push_back(s);
    t += trajectory_time_resolution;
  }

  double lon_objective_cost =
      compute_lon_trajectory_objective_cost(lon_trajectory, objective);

  double lon_jerk_cost = compute_lon_trajectory_jerk_cost(lon_trajectory);

  double lat_offset_cost =
      compute_lat_trajectory_offset_cost(lat_trajectory, s_values);

  return lon_objective_cost * weight_lon_objective +
         lon_jerk_cost * weight_lon_jerk + lat_offset_cost * weight_lat_offset;
}

double TrajectoryEvaluator::compute_lat_trajectory_offset_cost(
    const std::shared_ptr<Curve1d>& lat_trajectory,
    const std::vector<double>& s_values) const {
  double lat_s_max = lat_trajectory->param_length();
  double lat_offset_end = lat_trajectory->Evaluate(0, lat_s_max);
  double lat_offset_start = lat_trajectory->Evaluate(0, 0.0);

  double weight_same_side_offset = 1.0;
  double weight_opposite_side_offset = 10.0;

  double cost = 0.0;
  for (const auto& s : s_values) {
    double c = 0.0;
    if (s > lat_s_max) {
      // extend by same lateral offset
      c = lat_offset_end;
    } else {
      // get lateral offset by s
      c = lat_trajectory->Evaluate(0, s);
    }
    // Question(kechxu) what < 0.0 mean
    if (c * lat_offset_start < 0.0) {
      c = c * c * weight_opposite_side_offset;
    } else {
      c = c * c * weight_same_side_offset;
    }
    cost += c;
  }
  return cost;
}

double TrajectoryEvaluator::compute_lon_trajectory_jerk_cost(
    const std::shared_ptr<Curve1d>& lon_trajectory) const {
  double cost = 0.0;
  double t = 0.0;
  double lon_t_max = lon_trajectory->param_length();

  while (t < planned_trajectory_time) {
    double c = 0.0;
    if (t < lon_t_max) {
      c = lon_trajectory->Evaluate(3, t);
    } else {
      // when the time beyond the lon_trajectory parameter length,
      // the jerk is 0.0
    }
    cost += c * c;
    t = t + trajectory_time_resolution;
  }
  return cost;
}

double TrajectoryEvaluator::compute_lon_trajectory_objective_cost(
    const std::shared_ptr<Curve1d>& lon_trajectory,
    const PlanningTarget& objective) const {
  const LatticeSamplingConfig& lattice_sampling_config =
      objective.lattice_sampling_config();
  const LonSampleConfig& lon_sample_config =
      lattice_sampling_config.lon_sample_config();
  const LatSampleConfig& lat_sample_config =
      lattice_sampling_config.lat_sample_config();

  double s = lon_sample_config.lon_end_condition().s();
  double ds = lon_sample_config.lon_end_condition().ds();
  double dds = lon_sample_config.lon_end_condition().dds();

  if (objective.decision_type() == PlanningTarget::CRUISE) {
    // zero s target means cruise
    if (s <= std::numeric_limits<double>::epsilon()) {
      double target_speed = ds;
      double end_speed =
          lon_trajectory->Evaluate(1, lon_trajectory->param_length());

      double t_max = lon_trajectory->param_length();
      double t = 0.0;
      double cost = 0.0;

      while (t < planned_trajectory_time) {
        double c = 0.0;
        if (t < t_max) {
          c = target_speed - lon_trajectory->Evaluate(1, t);
        } else {
          c = target_speed - end_speed;
        }
        cost += std::fabs(c);
        t += trajectory_time_resolution;
      }
      return cost;
    } else {
      // Follow
      // apply the following 3 second rule.
      double target_s = s - 3.0 * ds;
      double end_s =
          lon_trajectory->Evaluate(0, lon_trajectory->param_length());

      double weight = 10.0;
      return (target_s - end_s) * weight;
    }
  } else {
    CHECK(objective.decision_type() == PlanningTarget::STOP);
    double target_s = s;
    double t_max = lon_trajectory->param_length();
    double end_s = lon_trajectory->Evaluate(0, t_max);

    double weight_overshoot = 100.0;
    double weight_undershoot = 1.0;

    double diff_s = target_s - end_s;
    if (diff_s < 0.0) {
      return weight_overshoot * (-diff_s);
    } else {
      return weight_undershoot * diff_s;
    }
  }
  return 0.0;
}

}  // namespace planning
}  // namespace apollo
