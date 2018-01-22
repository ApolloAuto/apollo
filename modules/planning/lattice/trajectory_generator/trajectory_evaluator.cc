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
    if (!ConstraintChecker1d::IsValidLongitudinalTrajectory(*lon_trajectory)) {
      continue;
    }
    for (const auto lat_trajectory : lat_trajectories) {
      if (!ConstraintChecker1d::IsValidLateralTrajectory(*lat_trajectory,
                                                         *lon_trajectory)) {
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
    ADEBUG << "Number of valid 1d trajectory pairs: " << cost_queue_.size();
  } else {
    ADEBUG << "Number of valid 1d trajectory pairs: "
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

std::vector<double>
TrajectoryEvaluator::top_trajectory_pair_component_cost() const {
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

  double weight_lon_travel = 5.0;
  double weight_lon_jerk = 1.0;
  double weight_lon_obstacle = 1.0;
  double weight_lat_offset = 1.0;

  double lon_travel_cost =
      compute_lon_objective_cost(lon_trajectory, planning_target);

  double lon_jerk_cost = compute_lon_comfort_cost(lon_trajectory);

  // for evaluating lateral trajectories
  std::vector<double> s_values;
  double s = 0.0;
  while (s < FLAGS_decision_horizon) {
    s_values.push_back(s);
    s += FLAGS_trajectory_space_resolution;
  }
  double lat_offset_cost = compute_lat_offset_cost(lat_trajectory, s_values);

  double lon_obstacle_cost = compute_lon_obstacle_cost(lon_trajectory);

  if (cost_components) {
    cost_components->push_back(lon_travel_cost);
    cost_components->push_back(lon_jerk_cost);
    cost_components->push_back(lon_obstacle_cost);
    cost_components->push_back(lat_offset_cost);
  }
  return lon_travel_cost * weight_lon_travel +
         lon_jerk_cost * weight_lon_jerk +
         lat_offset_cost * weight_lat_offset +
         lon_obstacle_cost * weight_lon_obstacle;
}

double TrajectoryEvaluator::compute_lat_offset_cost(
    const std::shared_ptr<Trajectory1d>& lat_trajectory,
    const std::vector<double>& s_values) const {
  double lat_offset_start = lat_trajectory->Evaluate(0, 0.0);
  double weight_same_side_offset = 1.0;
  double weight_opposite_side_offset = 10.0;
  double lat_offset_bound = 3.0;

  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (const auto& s : s_values) {
    double lat_offset = lat_trajectory->Evaluate(0, s);
    double cost = lat_offset / lat_offset_bound;
    if (lat_offset * lat_offset_start < 0.0) {
      cost_sqr_sum += cost * cost * weight_opposite_side_offset;
      cost_abs_sum += std::abs(cost) * weight_opposite_side_offset;
    } else {
      cost_sqr_sum += cost * cost * weight_same_side_offset;
      cost_abs_sum += std::abs(cost) * weight_same_side_offset;
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::compute_lon_comfort_cost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory) const {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  // TODO(all) need a new gflag for jerk_bound?
  double jerk_bound = FLAGS_longitudinal_jerk_upper_bound;
  for (double t = 0.0; t < FLAGS_trajectory_time_length;
       t += FLAGS_trajectory_time_resolution) {
    double jerk = lon_trajectory->Evaluate(3, t);
    double cost = jerk / jerk_bound;
    cost_sqr_sum += cost * cost;
    cost_abs_sum += std::abs(cost);
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::compute_lon_objective_cost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory,
    const PlanningTarget& planning_target) const {
  double weight_dist_travelled = 10.0;
  double weight_on_reference_speed = 1.0;
  double t_max = lon_trajectory->ParamLength();

  double dist_s =
      lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);

  double speed_cost_sqr_sum = 0.0;
  double speed_cost_abs_sum = 0.0;
  for (double t = 0.0; t < FLAGS_trajectory_time_length;
       t += FLAGS_trajectory_time_resolution) {
    double cost = planning_target.cruise_speed() -
                  lon_trajectory->Evaluate(1, t);
    speed_cost_sqr_sum += cost * cost;
    speed_cost_abs_sum += std::abs(cost);
  }
  double speed_cost = speed_cost_sqr_sum /
                      (speed_cost_abs_sum + FLAGS_lattice_epsilon);
  double dist_travelled_cost = 1.0 / (1.0 + dist_s);
  return (weight_on_reference_speed * speed_cost +
            weight_dist_travelled * dist_travelled_cost) /
         (weight_on_reference_speed + weight_dist_travelled);
}

// TODO(@all): consider putting pointer of reference_line_info and frame
// while constructing trajectory evaluator
double TrajectoryEvaluator::compute_lon_obstacle_cost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory) const {
  double start_time = 0.0;
  double end_time = FLAGS_trajectory_time_length;
  const auto& pt_intervals = pathtime_neighborhood_->GetPathBlockingIntervals(
      start_time, end_time, FLAGS_trajectory_time_resolution);
  // TODO(all) move buffer and sigma to gflags
  double buffer = 1.0;
  double sigma = 0.5;
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (std::size_t i = 0; i < pt_intervals.size(); ++i) {
    const auto& pt_interval = pt_intervals[i];
    if (pt_interval.empty()) {
      continue;
    }

    double t = start_time + i * FLAGS_trajectory_time_resolution;
    double traj_s = lon_trajectory->Evaluate(0, t);

    for (const auto& m : pt_interval) {
      double cost = 0.0;
      if (traj_s > m.first - buffer && traj_s < m.second + buffer) {
        cost = 1.0;
      } else if (traj_s < m.first) {
        double dist = traj_s - m.first + buffer;
        cost = std::exp(-dist * dist / (2.0 * sigma * sigma));
      } else if (traj_s > m.second) {
        double dist = m.second + buffer - traj_s;
        cost = std::exp(-dist * dist / (2.0 * sigma * sigma));
      }
      cost_sqr_sum += cost * cost;
      cost_abs_sum += std::abs(cost);
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
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
