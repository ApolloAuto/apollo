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

#include "modules/planning/lattice/trajectory_generator/trajectory_evaluator.h"

#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/lattice_params.h"
#include "modules/planning/constraint_checker/constraint_checker1d.h"

namespace apollo {
namespace planning {

using Trajectory1d = Curve1d;

TrajectoryEvaluator::TrajectoryEvaluator(
    const PlanningTarget& planning_target,
    const std::vector<std::shared_ptr<Trajectory1d>>& lon_trajectories,
    const std::vector<std::shared_ptr<Trajectory1d>>& lat_trajectories,
    bool is_auto_tuning,
    std::shared_ptr<PathTimeNeighborhood> pathtime_neighborhood)
    : is_auto_tuning_(is_auto_tuning),
      pathtime_neighborhood_(pathtime_neighborhood) {
  for (const auto lon_trajectory : lon_trajectories) {
    if (!ConstraintChecker1d::IsValidLongitudinalTrajectory(
            *lon_trajectory)) {
      continue;
    }
    for (const auto lat_trajectory : lat_trajectories) {
      if (!ConstraintChecker1d::IsValidLateralTrajectory(
              *lat_trajectory, *lon_trajectory)) {
        continue;
      }
      if (!is_auto_tuning_) {
        double cost =
            evaluate(planning_target, lon_trajectory, lat_trajectory, nullptr);
        cost_queue_.push(PairCost({lon_trajectory, lat_trajectory}, cost));
      } else {
        std::vector<double> cost_components;
        double cost = evaluate(planning_target, lon_trajectory, lat_trajectory,
                               &cost_components);
        cost_queue_with_components_.push(PairCostWithComponents(
            {lon_trajectory, lat_trajectory}, {cost_components, cost}));
      }
    }
  }
  if (!is_auto_tuning_) {
    AINFO << "Number of valid 1d trajectory pairs:\t" << cost_queue_.size();
  } else {
    AINFO << "Number of valid 1d trajectory pairs:\t"
          << cost_queue_with_components_.size();
  }
}

bool TrajectoryEvaluator::has_more_trajectory_pairs() const {
  if (!is_auto_tuning_) {
    return !cost_queue_.empty();
  } else {
    return !cost_queue_with_components_.empty();
  }
}

std::size_t TrajectoryEvaluator::num_of_trajectory_pairs() const {
  if (!is_auto_tuning_) {
    return cost_queue_.size();
  } else {
    return cost_queue_with_components_.empty();
  }
}

std::pair<std::shared_ptr<Trajectory1d>, std::shared_ptr<Trajectory1d>>
TrajectoryEvaluator::next_top_trajectory_pair() {
  CHECK(has_more_trajectory_pairs() == true);
  if (!is_auto_tuning_) {
    auto top = cost_queue_.top();
    cost_queue_.pop();
    return top.first;
  } else {
    auto top = cost_queue_with_components_.top();
    cost_queue_with_components_.pop();
    return top.first;
  }
}

double TrajectoryEvaluator::top_trajectory_pair_cost() const {
  if (!is_auto_tuning_) {
    return cost_queue_.top().second;
  } else {
    return cost_queue_with_components_.top().second.second;
  }
}

std::vector<double> TrajectoryEvaluator::top_trajectory_pair_component_cost()
    const {
  CHECK(is_auto_tuning_);
  return cost_queue_with_components_.top().second.first;
}

double TrajectoryEvaluator::evaluate(
    const PlanningTarget& planning_target,
    const std::shared_ptr<Trajectory1d>& lon_trajectory,
    const std::shared_ptr<Trajectory1d>& lat_trajectory,
    std::vector<double>* cost_components) const {
  // currently consider three costs:
  // 1. the cost of jerk, currently only consider longitudinal jerk.
  // 2. the cost of time to achieve the objective
  // 3. the cost of failure to achieve the planning objective.

  double weight_lon_travel = 1000.0;
  double weight_lon_jerk = 1.0;
  double weight_lon_obstacle = 100.0;
  double weight_lat_offset = 1.0;

  double t = 0.0;
  double t_max = lon_trajectory->ParamLength();
  double s_max = lon_trajectory->Evaluate(0, t_max);
  double v_end = lon_trajectory->Evaluate(1, t_max);

  double lon_travel_cost =
      compute_lon_objective_cost(lon_trajectory, planning_target);

  double lon_jerk_cost = compute_lon_comfort_cost(lon_trajectory);

  // for evaluating lateral trajectories
  std::vector<double> s_values;
  double s = 0.0;
  while (s < decision_horizon) {
    s_values.push_back(s);
    s += trajectory_space_resolution;
  }
  double lat_offset_cost = compute_lat_offset_cost(lat_trajectory, s_values);

  double lon_obstacle_cost = compute_lon_obstacle_cost(lon_trajectory);

  if (cost_components) {
    cost_components->push_back(lon_travel_cost);
    cost_components->push_back(lon_jerk_cost);
    cost_components->push_back(lon_obstacle_cost);
    cost_components->push_back(lat_offset_cost);
  }
  return lon_travel_cost * weight_lon_travel + lon_jerk_cost * weight_lon_jerk +
         lat_offset_cost * weight_lat_offset +
         lon_obstacle_cost * weight_lon_obstacle;
}

double TrajectoryEvaluator::compute_lat_offset_cost(
    const std::shared_ptr<Trajectory1d>& lat_trajectory,
    const std::vector<double>& s_values) const {
  double lat_s_max = lat_trajectory->ParamLength();
  double lat_offset_end = lat_trajectory->Evaluate(0, lat_s_max);
  double lat_offset_start = lat_trajectory->Evaluate(0, 0.0);

  double weight_same_side_offset = 1.0;
  double weight_opposite_side_offset = 10.0;

  double cost = 0.0;
  for (const auto& s : s_values) {
    double c = lat_trajectory->Evaluate(0, s);
    if (c * lat_offset_start < 0.0) {
      c = c * c * weight_opposite_side_offset;
    } else {
      c = c * c * weight_same_side_offset;
    }
    cost += c;
  }
  return cost;
}

double TrajectoryEvaluator::compute_lon_comfort_cost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory) const {
  double cost = 0.0;
  double t = 0.0;

  while (t < planned_trajectory_time) {
    double c = lon_trajectory->Evaluate(3, t);
    cost += c * c;
    t = t + trajectory_time_resolution;
  }
  return cost;
}

double TrajectoryEvaluator::compute_lon_objective_cost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory,
    const PlanningTarget& planning_target) const {
  double weight_dist_travelled = 10.0;
  double weight_on_reference_speed = 1.0;
  double t_max = lon_trajectory->ParamLength();

  double dist_s =
      lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);

  double cost = 0.0;
  double t = 0.0;
  while (t < planned_trajectory_time) {
    double c = planning_target.cruise_speed() - lon_trajectory->Evaluate(1, t);
    cost += std::fabs(c);
    t += trajectory_time_resolution;
  }
  cost *= weight_on_reference_speed;
  cost += weight_dist_travelled * 1.0 / (1.0 + dist_s);
  return cost;
}

// TODO(@all): consider putting pointer of reference_line_info and frame
// while constructing trajectory evaluator
double TrajectoryEvaluator::compute_lon_obstacle_cost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory) const {
  double start_time = 0.0;
  double end_time = planned_trajectory_time;
  const auto& pt_intervals = pathtime_neighborhood_->GetPathBlockingIntervals(
      start_time, end_time, trajectory_time_resolution);

  double obstacle_cost = 0.0;
  for (std::size_t i = 0; i < pt_intervals.size(); ++i) {
    const auto& pt_interval = pt_intervals[i];
    if (pt_interval.empty()) {
      continue;
    }

    double t = start_time + i * trajectory_time_resolution;
    double traj_s = lon_trajectory->Evaluate(0, t);

    for (const auto& m : pt_interval) {
      double mid_s = (m.first + m.second) * 0.5;
      obstacle_cost += 1.0 / std::exp(std::abs(traj_s - mid_s));
    }
  }
  return obstacle_cost;
}

std::vector<double> TrajectoryEvaluator::evaluate_per_lonlat_trajectory(
    const PlanningTarget& planning_target,
    const std::vector<apollo::common::SpeedPoint> st_points,
    const std::vector<apollo::common::FrenetFramePoint> sl_points) {
  std::vector<double> ret;
  return ret;
}

}  // namespace planning
}  // namespace apollo
