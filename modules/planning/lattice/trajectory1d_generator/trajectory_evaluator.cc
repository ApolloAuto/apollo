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

using Trajectory1d = Curve1d;

TrajectoryEvaluator::TrajectoryEvaluator(
    const PlanningTarget& planning_target,
    const std::vector<std::shared_ptr<Trajectory1d>>& lon_trajectories,
    const std::vector<std::shared_ptr<Trajectory1d>>& lat_trajectories) {

  for (const auto lon_trajectory : lon_trajectories) {
    if (!LatticeConstraintChecker::IsValidLongitudinalTrajectory(
            *lon_trajectory)) {
      continue;
    }
    for (const auto lat_trajectory : lat_trajectories) {
      if (!LatticeConstraintChecker::IsValidLateralTrajectory(
              *lat_trajectory, *lon_trajectory)) {
        continue;
      }
      double cost =
          evaluate(planning_target, lon_trajectory, lat_trajectory);
      cost_queue_.push(
          PairCost({lon_trajectory, lat_trajectory}, cost));
    }
  }
}

bool TrajectoryEvaluator::has_more_trajectory_pairs() const {
  return !cost_queue_.empty();
}

size_t TrajectoryEvaluator::num_of_trajectory_pairs() const {
  return cost_queue_.size();
}

std::pair<std::shared_ptr<Trajectory1d>, std::shared_ptr<Trajectory1d>>
TrajectoryEvaluator::next_top_trajectory_pair() {
  CHECK(has_more_trajectory_pairs() == true);
  auto top = cost_queue_.top();
  cost_queue_.pop();
  return top.first;
}

double TrajectoryEvaluator::top_trajectory_pair_cost() const {
  return cost_queue_.top().second;
}

double TrajectoryEvaluator::evaluate(
    const PlanningTarget& planning_target,
    const std::shared_ptr<Trajectory1d>& lon_trajectory,
    const std::shared_ptr<Trajectory1d>& lat_trajectory) const {
  // currently consider three costs:
  // 1. the cost of jerk, currently only consider longitudinal jerk.
  // 2. the cost of time to achieve the objective
  // 3. the cost of failure to achieve the planning objective.

  // these numbers are for temporary use; needs to be tuned later.
  double weight_lon_jerk = 1.0;
  double weight_lon_objective = 3000.0;
  double weight_lat_offset = 10.0;

  double t = 0.0;
  double t_max = lon_trajectory->ParamLength();
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
      compute_lon_trajectory_objective_cost(lon_trajectory, planning_target);

  double lon_jerk_cost = compute_lon_trajectory_jerk_cost(lon_trajectory);

  double lat_offset_cost =
      compute_lat_trajectory_offset_cost(lat_trajectory, s_values);

  return lon_objective_cost * weight_lon_objective +
         lon_jerk_cost * weight_lon_jerk + lat_offset_cost * weight_lat_offset;
}

double TrajectoryEvaluator::compute_lat_trajectory_offset_cost(
    const std::shared_ptr<Trajectory1d>& lat_trajectory,
    const std::vector<double>& s_values) const {
  double lat_s_max = lat_trajectory->ParamLength();
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
    const std::shared_ptr<Trajectory1d>& lon_trajectory) const {
  double cost = 0.0;
  double t = 0.0;
  double lon_t_max = lon_trajectory->ParamLength();

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
    const std::shared_ptr<Trajectory1d>& lon_trajectory,
    const PlanningTarget& planning_target) const {

    // zero s target means cruise
//    if (s <= std::numeric_limits<double>::epsilon()) {
//      double target_speed = ds;
//      double end_speed =
//          lon_trajectory->Evaluate(1, lon_trajectory->ParamLength());
//
//      double t_max = lon_trajectory->ParamLength();
//      double t = 0.0;
//      double cost = 0.0;
//
//      while (t < planned_trajectory_time) {
//        double c = 0.0;
//        if (t < t_max) {
//          c = target_speed - lon_trajectory->Evaluate(1, t);
//        } else {
//          c = target_speed - end_speed;
//        }
//        cost += std::fabs(c);
//        t += trajectory_time_resolution;
//      }
//      return cost;
//    } else {
//      // Follow
//      // apply the following 3 second rule.
//      double target_s = s - 3.0 * ds;
//      double end_s =
//          lon_trajectory->Evaluate(0, lon_trajectory->ParamLength());
//
//      double weight = 10.0;
//      return (target_s - end_s) * weight;
//    }
  double weight_dist_s = 10.0;
  double target_speed = planning_target.cruise_speed();
  double end_speed = lon_trajectory->Evaluate(1,
      lon_trajectory->ParamLength());

  double t_max = lon_trajectory->ParamLength();
  double t = 0.0;
  double s_max = lon_trajectory->Evaluate(0, planned_trajectory_time);
  double s_min = lon_trajectory->Evaluate(0, 0.0);
  double dist_s = s_max -s_min;
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
  cost += weight_dist_s * 1.0 / (1.0 + dist_s);
  return cost;

}

}  // namespace planning
}  // namespace apollo
