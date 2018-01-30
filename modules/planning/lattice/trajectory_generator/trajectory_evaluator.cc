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
            Evaluate(planning_target, lon_trajectory, lat_trajectory, nullptr);
        cost_queue_.push(PairCost({lon_trajectory, lat_trajectory}, cost));
      } else {
        std::vector<double> cost_components;
        double cost = Evaluate(planning_target, lon_trajectory, lat_trajectory,
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
    return cost_queue_with_components_.size();
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

double TrajectoryEvaluator::Evaluate(
    const PlanningTarget& planning_target,
    const std::shared_ptr<Trajectory1d>& lon_trajectory,
    const std::shared_ptr<Trajectory1d>& lat_trajectory,
    std::vector<double>* cost_components) const {
  // Costs:
  // 1. Cost to achieve the objective
  // 2. Cost of logitudinal jerk
  // 3. Cost of logitudinal collision
  // 4. Cost of lateral offsets

  // Longitudinal costs
  double lon_travel_cost = LonObjectiveCost(lon_trajectory, planning_target);
  double lon_jerk_cost = LonComfortCost(lon_trajectory);
  double lon_collision_cost = LonCollisionCost(lon_trajectory);

  std::vector<double> s_values;
  for (double s = 0.0; s < FLAGS_decision_horizon;
       s += FLAGS_trajectory_space_resolution) {
    s_values.push_back(s);
  }

  // Lateral costs
  double lat_offset_cost = LatOffsetCost(lat_trajectory, s_values);

  if (cost_components) {
    cost_components->push_back(lon_travel_cost);
    cost_components->push_back(lon_jerk_cost);
    cost_components->push_back(lon_collision_cost);
    cost_components->push_back(lat_offset_cost);
  }

  return lon_travel_cost * FLAGS_weight_lon_travel +
         lon_jerk_cost * FLAGS_weight_lon_jerk +
         lon_collision_cost * FLAGS_weight_lon_collision +
         lat_offset_cost * FLAGS_weight_lat_offset;
}

double TrajectoryEvaluator::LatOffsetCost(
    const std::shared_ptr<Trajectory1d>& lat_trajectory,
    const std::vector<double>& s_values) const {
  double lat_offset_start = lat_trajectory->Evaluate(0, 0.0);
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (const auto& s : s_values) {
    double lat_offset = lat_trajectory->Evaluate(0, s);
    double cost = lat_offset / FLAGS_lat_offset_bound;
    if (lat_offset * lat_offset_start < 0.0) {
      cost_sqr_sum += cost * cost * FLAGS_weight_opposite_side_offset;
      cost_abs_sum += std::abs(cost) * FLAGS_weight_opposite_side_offset;
    } else {
      cost_sqr_sum += cost * cost * FLAGS_weight_same_side_offset;
      cost_abs_sum += std::abs(cost) * FLAGS_weight_same_side_offset;
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

// AutoTuning LatOffSetCost from discretized trajectory
double TrajectoryEvaluator::LatOffsetCost(
  const std::vector<apollo::common::FrenetFramePoint> sl_points) const {
  CHECK_GT(sl_points.size(), 0);
  if (sl_points.size() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  double lat_offset_start = sl_points[0].l();
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (const apollo::common::FrenetFramePoint& sl_point : sl_points) {
    double lat_offset = sl_point.l();
    double cost = lat_offset / FLAGS_lat_offset_bound;
    if (lat_offset * lat_offset_start < 0.0) {
      cost_sqr_sum += cost * cost * FLAGS_weight_opposite_side_offset;
      cost_abs_sum += std::abs(cost) * FLAGS_weight_opposite_side_offset;
    } else {
      cost_sqr_sum += cost * cost * FLAGS_weight_same_side_offset;
      cost_abs_sum += std::abs(cost) * FLAGS_weight_same_side_offset;
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::LonComfortCost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory) const {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (double t = 0.0; t < FLAGS_trajectory_time_length;
       t += FLAGS_trajectory_time_resolution) {
    double jerk = lon_trajectory->Evaluate(3, t);
    double cost = jerk / FLAGS_longitudinal_jerk_upper_bound;
    cost_sqr_sum += cost * cost;
    cost_abs_sum += std::abs(cost);
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

// AutoTuning LonComfortCost from discretized trajectory
double TrajectoryEvaluator::LonComfortCost(
  const std::vector<apollo::common::SpeedPoint> st_points) const {
  CHECK_GT(st_points.size(), 1);
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (size_t i = 0; i < st_points.size()-1; ++i) {
    double dds1 = st_points[i].a();
    double dds2 = st_points[i+1].a();
    double t1 = st_points[i].t();
    double t2 = st_points[i].t();
    if (std::abs(t1 - t2) <= FLAGS_lattice_epsilon) {
      continue;
    }
    double jerk =  (dds2 - dds1) / (t2 - t1);
    double cost = jerk / FLAGS_longitudinal_jerk_upper_bound;
    cost_sqr_sum += cost * cost;
    cost_abs_sum += std::abs(cost);
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::LonObjectiveCost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory,
    const PlanningTarget& planning_target) const {
  double t_max = lon_trajectory->ParamLength();
  double dist_s =
      lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);

  double speed_cost_sqr_sum = 0.0;
  double speed_cost_abs_sum = 0.0;
  for (double t = 0.0; t < FLAGS_trajectory_time_length;
       t += FLAGS_trajectory_time_resolution) {
    double cost =
        planning_target.cruise_speed() - lon_trajectory->Evaluate(1, t);
    speed_cost_sqr_sum += cost * cost;
    speed_cost_abs_sum += std::abs(cost);
  }
  double speed_cost =
      speed_cost_sqr_sum / (speed_cost_abs_sum + FLAGS_lattice_epsilon);
  double dist_travelled_cost = 1.0 / (1.0 + dist_s);
  return (speed_cost * FLAGS_weight_target_speed +
          dist_travelled_cost * FLAGS_weight_dist_travelled) /
         (FLAGS_weight_target_speed + FLAGS_weight_dist_travelled);
}

// AutoTuning LonObjectiveCost from discretized trajectory
double TrajectoryEvaluator::LonObjectiveCost(
  const std::vector<apollo::common::SpeedPoint> st_points,
  const PlanningTarget& planning_target) const {
  CHECK_GT(st_points.size(), 1);
  double dist_s = st_points[st_points.size()-1].s() -
                  st_points[0].s();
  double speed_cost_sqr_sum = 0.0;
  double speed_cost_abs_sum = 0.0;
  for (const apollo::common::SpeedPoint& st_point : st_points) {
    double cost =  planning_target.cruise_speed() - st_point.v();
    speed_cost_sqr_sum += cost * cost;
    speed_cost_abs_sum += std::abs(cost);
  }
  double speed_cost = speed_cost_sqr_sum /
                      (speed_cost_abs_sum + FLAGS_lattice_epsilon);
  double dist_travelled_cost = 1.0 / (1.0 + dist_s);
  return (speed_cost * FLAGS_weight_target_speed +
            dist_travelled_cost * FLAGS_weight_dist_travelled) /
         (FLAGS_weight_target_speed + FLAGS_weight_dist_travelled);
}

// TODO(all): consider putting pointer of reference_line_info and frame
// while constructing trajectory evaluator
double TrajectoryEvaluator::LonCollisionCost(
    const std::shared_ptr<Trajectory1d>& lon_trajectory) const {
  double start_time = 0.0;
  double end_time = FLAGS_trajectory_time_length;
  const auto& pt_intervals = pathtime_neighborhood_->GetPathBlockingIntervals(
      start_time, end_time, FLAGS_trajectory_time_resolution);
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (std::size_t i = 0; i < pt_intervals.size(); ++i) {
    const auto& pt_interval = pt_intervals[i];
    if (pt_interval.empty()) {
      continue;
    }
    double t = start_time + i * FLAGS_trajectory_time_resolution;
    double traj_s = lon_trajectory->Evaluate(0, t);
    double sigma = FLAGS_lon_collision_cost_std;
    for (const auto& m : pt_interval) {
      double cost = 0.0;
      if (traj_s > m.first - FLAGS_lon_collision_yield_buffer &&
          traj_s < m.second + FLAGS_lon_collision_overtake_buffer) {
        cost = 1.0;
      } else if (traj_s < m.first) {
        double dist = (m.first - FLAGS_lon_collision_yield_buffer) - traj_s;
        cost = std::exp(-dist * dist / (2.0 * sigma * sigma));
      } else if (traj_s > m.second) {
        double dist = traj_s - (m.second + FLAGS_lon_collision_overtake_buffer);
        cost = std::exp(-dist * dist / (2.0 * sigma * sigma));
      }
      cost_sqr_sum += cost * cost;
      cost_abs_sum += std::abs(cost);
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

// AutoTuning LonCollisionCost
double TrajectoryEvaluator::LonCollisionCost(
    const std::vector<apollo::common::SpeedPoint> st_points) const {
  double start_time = 0.0;
  double end_time = FLAGS_trajectory_time_length;
  const auto& pt_intervals = pathtime_neighborhood_->GetPathBlockingIntervals(
      start_time, end_time, FLAGS_trajectory_time_resolution);
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (std::size_t i = 0; i < pt_intervals.size(); ++i) {
    const auto& pt_interval = pt_intervals[i];
    if (pt_interval.empty()) {
      continue;
    }
    double t = start_time + i * FLAGS_trajectory_time_resolution;
    double traj_s = std::numeric_limits<double>::infinity();
    if (!InterpolateDenseStPoints(st_points, t, &traj_s)) {
      AERROR << "AutoTuning LonCollisionCost InterpolateDenseStPoints Error";
      return traj_s;
    }
    double sigma = FLAGS_lon_collision_cost_std;
    for (const auto& m : pt_interval) {
      double cost = 0.0;
      if (traj_s > m.first - FLAGS_lon_collision_buffer &&
          traj_s < m.second + FLAGS_lon_collision_buffer) {
        cost = 1.0;
      } else if (traj_s < m.first) {
        double dist = traj_s - m.first + FLAGS_lon_collision_buffer;
        cost = std::exp(-dist * dist / (2.0 * sigma * sigma));
      } else if (traj_s > m.second) {
        double dist = m.second + FLAGS_lon_collision_buffer - traj_s;
        cost = std::exp(-dist * dist / (2.0 * sigma * sigma));
      }
      cost_sqr_sum += cost * cost;
      cost_abs_sum += std::abs(cost);
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

bool TrajectoryEvaluator::InterpolateDenseStPoints(
  const std::vector<apollo::common::SpeedPoint> st_points,
  double t, double *traj_s) const {
  CHECK_GT(st_points.size(), 1);
  if (t < st_points[0].t() || t > st_points[st_points.size()-1].t()) {
    AERROR << "AutoTuning InterpolateDenseStPoints Error";
    return false;
  }
  for (uint i = 1; i < st_points.size(); ++i) {
    if (t <= st_points[i].t()) {
      *traj_s = st_points[i].t();
      return true;
    }
  }
  return false;
}

double TrajectoryEvaluator::evaluate_per_lonlat_trajectory(
    const PlanningTarget& planning_target,
    const std::vector<apollo::common::SpeedPoint> st_points,
    const std::vector<apollo::common::FrenetFramePoint> sl_points,
    std::vector<double>* cost_components) {

  // Costs:
  // 1. Cost to achieve the objective
  // 2. Cost of logitudinal jerk
  // 3. Cost of logitudinal collision
  // 4. Cost of lateral offsets

  // Longitudinal costs
  double lon_travel_cost = LonObjectiveCost(st_points, planning_target);
  double lon_jerk_cost = LonComfortCost(st_points);
  double lon_collision_cost = LonCollisionCost(st_points);

  // Lateral costs
  double lat_offset_cost = LatOffsetCost(sl_points);

  if (cost_components) {
    cost_components->push_back(lon_travel_cost);
    cost_components->push_back(lon_jerk_cost);
    cost_components->push_back(lon_collision_cost);
    cost_components->push_back(lat_offset_cost);
  }

  return lon_travel_cost * FLAGS_weight_lon_travel +
         lon_jerk_cost * FLAGS_weight_lon_jerk +
         lon_collision_cost * FLAGS_weight_lon_collision +
         lat_offset_cost * FLAGS_weight_lat_offset;
}

}  // namespace planning
}  // namespace apollo
