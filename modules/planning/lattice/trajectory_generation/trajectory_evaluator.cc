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

#include "modules/planning/lattice/trajectory_generation/trajectory_evaluator.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker1d.h"
#include "modules/planning/lattice/trajectory1d/piecewise_acceleration_trajectory1d.h"

namespace apollo {
namespace planning {

using Trajectory1d = Curve1d;
using apollo::common::math::PathMatcher;
using apollo::common::FrenetFramePoint;
using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using Trajectory1dPair =
    std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>;
using CostComponentsPair = std::pair<std::vector<double>, double>;

using PtrTrajectory1d = std::shared_ptr<Trajectory1d>;

TrajectoryEvaluator::TrajectoryEvaluator(
    const std::array<double, 3>& init_s,
    const PlanningTarget& planning_target,
    const std::vector<PtrTrajectory1d>& lon_trajectories,
    const std::vector<PtrTrajectory1d>& lat_trajectories,
    std::shared_ptr<PathTimeGraph> path_time_graph,
    std::shared_ptr<std::vector<PathPoint>> reference_line)
    : path_time_graph_(path_time_graph),
      reference_line_(reference_line),
      init_s_(init_s) {
  const double start_time = 0.0;
  const double end_time = FLAGS_trajectory_time_length;
  path_time_intervals_ = path_time_graph_->GetPathBlockingIntervals(
      start_time, end_time, FLAGS_trajectory_time_resolution);

  reference_s_dot_ = ComputeLongitudinalGuideVelocity(planning_target);

  // if we have a stop point along the reference line,
  // filter out the lon. trajectories that pass the stop point.
  double stop_point = std::numeric_limits<double>::max();
  if (planning_target.has_stop_point()) {
    stop_point = planning_target.stop_point().s();
  }
  for (const auto& lon_trajectory : lon_trajectories) {
    double lon_end_s = lon_trajectory->Evaluate(0, end_time);
    if (init_s[0] < stop_point &&
        lon_end_s + FLAGS_lattice_stop_buffer > stop_point) {
      continue;
    }

    if (!ConstraintChecker1d::IsValidLongitudinalTrajectory(*lon_trajectory)) {
      continue;
    }
    for (const auto& lat_trajectory : lat_trajectories) {
      /**
       * The validity of the code needs to be verified.
      if (!ConstraintChecker1d::IsValidLateralTrajectory(*lat_trajectory,
                                                         *lon_trajectory)) {
        continue;
      }
      */
      if (!FLAGS_enable_auto_tuning) {
        double cost = Evaluate(planning_target, lon_trajectory, lat_trajectory);
        cost_queue_.emplace(Trajectory1dPair(lon_trajectory, lat_trajectory),
                            cost);
      } else {
        std::vector<double> cost_components;
        double cost = Evaluate(planning_target, lon_trajectory, lat_trajectory,
                               &cost_components);
        cost_queue_with_components_.emplace(
            Trajectory1dPair(lon_trajectory, lat_trajectory),
            CostComponentsPair(cost_components, cost));
      }
    }
  }
  if (!FLAGS_enable_auto_tuning) {
    ADEBUG << "Number of valid 1d trajectory pairs: " << cost_queue_.size();
  } else {
    ADEBUG << "Number of valid 1d trajectory pairs: "
           << cost_queue_with_components_.size();
  }
}

bool TrajectoryEvaluator::has_more_trajectory_pairs() const {
  if (!FLAGS_enable_auto_tuning) {
    return !cost_queue_.empty();
  } else {
    return !cost_queue_with_components_.empty();
  }
}

std::size_t TrajectoryEvaluator::num_of_trajectory_pairs() const {
  if (!FLAGS_enable_auto_tuning) {
    return cost_queue_.size();
  } else {
    return cost_queue_with_components_.size();
  }
}

std::pair<PtrTrajectory1d, PtrTrajectory1d>
TrajectoryEvaluator::next_top_trajectory_pair() {
  CHECK(has_more_trajectory_pairs() == true);
  if (!FLAGS_enable_auto_tuning) {
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
  if (!FLAGS_enable_auto_tuning) {
    return cost_queue_.top().second;
  } else {
    return cost_queue_with_components_.top().second.second;
  }
}

std::vector<double> TrajectoryEvaluator::top_trajectory_pair_component_cost()
    const {
  CHECK(FLAGS_enable_auto_tuning);
  return cost_queue_with_components_.top().second.first;
}

double TrajectoryEvaluator::Evaluate(
    const PlanningTarget& planning_target,
    const PtrTrajectory1d& lon_trajectory,
    const PtrTrajectory1d& lat_trajectory,
    std::vector<double>* cost_components) const {
  // Costs:
  // 1. Cost of missing the objective, e.g., cruise, stop, etc.
  // 2. Cost of logitudinal jerk
  // 3. Cost of logitudinal collision
  // 4. Cost of lateral offsets
  // 5. Cost of lateral comfort

  // Longitudinal costs
  double lon_objective_cost =
      LonObjectiveCost(lon_trajectory, planning_target, reference_s_dot_);

  double lon_jerk_cost = LonComfortCost(lon_trajectory);

  double lon_collision_cost = LonCollisionCost(lon_trajectory);

  double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);

  // decides the longitudinal evaluation horizon for lateral trajectories.
  double evaluation_horizon =
      std::min(FLAGS_decision_horizon,
               lon_trajectory->Evaluate(0, lon_trajectory->ParamLength()));
  std::vector<double> s_values;
  for (double s = 0.0; s < evaluation_horizon;
       s += FLAGS_trajectory_space_resolution) {
    s_values.emplace_back(s);
  }

  // Lateral costs
  double lat_offset_cost = LatOffsetCost(lat_trajectory, s_values);

  double lat_comfort_cost = LatComfortCost(lon_trajectory, lat_trajectory);

  if (cost_components != nullptr) {
    cost_components->emplace_back(lon_objective_cost);
    cost_components->emplace_back(lon_jerk_cost);
    cost_components->emplace_back(lon_collision_cost);
    cost_components->emplace_back(lat_offset_cost);
  }

  return lon_objective_cost * FLAGS_weight_lon_objective +
         lon_jerk_cost * FLAGS_weight_lon_jerk +
         lon_collision_cost * FLAGS_weight_lon_collision +
         centripetal_acc_cost * FLAGS_weight_centripetal_acceleration +
         lat_offset_cost * FLAGS_weight_lat_offset +
         lat_comfort_cost * FLAGS_weight_lat_comfort;
}

double TrajectoryEvaluator::EvaluateDiscreteTrajectory(
    const PlanningTarget& planning_target,
    const std::vector<SpeedPoint>& st_points,
    const std::vector<FrenetFramePoint>& sl_points,
    std::vector<double>* cost_components) {
  CHECK_NOTNULL(cost_components);

  double lat_offset_cost = LatOffsetCost(sl_points);
  double lat_comfort_cost = LatComfortCost(sl_points);
  double lon_comfort_cost = LonComfortCost(st_points);
  double lon_collision_cost = LonCollisionCost(st_points);
  double lon_objective_cost =
      LonObjectiveCost(st_points, planning_target, reference_s_dot_);
  double centripetal_acc_cost = CentripetalAccelerationCost(st_points);

  cost_components->emplace_back(lat_offset_cost);
  cost_components->emplace_back(lat_comfort_cost);
  cost_components->emplace_back(lon_comfort_cost);
  cost_components->emplace_back(lon_collision_cost);
  cost_components->emplace_back(lon_objective_cost);
  cost_components->emplace_back(centripetal_acc_cost);

  return lon_objective_cost * FLAGS_weight_lon_objective +
         lon_comfort_cost * FLAGS_weight_lon_jerk +
         lon_collision_cost * FLAGS_weight_lon_collision +
         centripetal_acc_cost * FLAGS_weight_centripetal_acceleration +
         lat_offset_cost * FLAGS_weight_lat_offset +
         lat_comfort_cost * FLAGS_weight_lat_comfort;
}

double TrajectoryEvaluator::LatOffsetCost(
    const PtrTrajectory1d& lat_trajectory,
    const std::vector<double>& s_values) const {
  double lat_offset_start = lat_trajectory->Evaluate(0, 0.0);
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (const auto& s : s_values) {
    double lat_offset = lat_trajectory->Evaluate(0, s);
    double cost = lat_offset / FLAGS_lat_offset_bound;
    if (lat_offset * lat_offset_start < 0.0) {
      cost_sqr_sum += cost * cost * FLAGS_weight_opposite_side_offset;
      cost_abs_sum += std::fabs(cost) * FLAGS_weight_opposite_side_offset;
    } else {
      cost_sqr_sum += cost * cost * FLAGS_weight_same_side_offset;
      cost_abs_sum += std::fabs(cost) * FLAGS_weight_same_side_offset;
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::LatOffsetCost(
    const std::vector<FrenetFramePoint> sl_points) const {
  if (sl_points.size() == 0) {
    return 0.0;
  }
  double lat_offset_start = sl_points[0].l();
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (const common::FrenetFramePoint& sl_point : sl_points) {
    double lat_offset = sl_point.l();
    double cost = lat_offset / FLAGS_lat_offset_bound;
    if (lat_offset * lat_offset_start < 0.0) {
      cost_sqr_sum += cost * cost * FLAGS_weight_opposite_side_offset;
      cost_abs_sum += std::fabs(cost) * FLAGS_weight_opposite_side_offset;
    } else {
      cost_sqr_sum += cost * cost * FLAGS_weight_same_side_offset;
      cost_abs_sum += std::fabs(cost) * FLAGS_weight_same_side_offset;
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::LatComfortCost(
    const PtrTrajectory1d& lon_trajectory,
    const PtrTrajectory1d& lat_trajectory) const {
  double max_cost = 0.0;
  for (double t = 0.0; t < FLAGS_trajectory_time_length;
       t += FLAGS_trajectory_time_resolution) {
    double s = lon_trajectory->Evaluate(0, t);
    double s_dot = lon_trajectory->Evaluate(1, t);
    double s_dotdot = lon_trajectory->Evaluate(2, t);
    double l_prime = lat_trajectory->Evaluate(1, s);
    double l_primeprime = lat_trajectory->Evaluate(2, s);
    double cost = l_primeprime * s_dot * s_dot + l_prime * s_dotdot;
    max_cost = std::max(max_cost, std::fabs(cost));
  }
  return max_cost;
}

double TrajectoryEvaluator::LatComfortCost(
    const std::vector<FrenetFramePoint>& sl_points) const {
  if (sl_points.size() < 2) {
    return 0.0;
  }
  double dt = FLAGS_trajectory_time_resolution;
  double max_cost = 0.0;
  for (std::size_t i = 1; i + 1 < sl_points.size(); ++i) {
    double s_dot = (sl_points[i + 1].s() - sl_points[i].s()) / dt;
    double s_dotdot =
        (sl_points[i + 1].s() - 2.0 * sl_points[i].s() + sl_points[i - 1].s()) /
        (dt * dt);
    double l_prime = sl_points[i].dl();
    double l_primeprime = sl_points[i].ddl();
    double cost = l_primeprime * s_dot * s_dot + l_prime * s_dotdot;
    max_cost = std::max(max_cost, std::fabs(cost));
  }
  return max_cost;
}

double TrajectoryEvaluator::LonComfortCost(
    const PtrTrajectory1d& lon_trajectory) const {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (double t = 0.0; t < FLAGS_trajectory_time_length;
       t += FLAGS_trajectory_time_resolution) {
    double jerk = lon_trajectory->Evaluate(3, t);
    double cost = jerk / FLAGS_longitudinal_jerk_upper_bound;
    cost_sqr_sum += cost * cost;
    cost_abs_sum += std::fabs(cost);
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::LonComfortCost(
    const std::vector<SpeedPoint>& st_points) const {
  if (st_points.size() < 2) {
    return 0.0;
  }
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (std::size_t i = 0; i < st_points.size() - 1; ++i) {
    double dds1 = st_points[i].a();
    double dds2 = st_points[i + 1].a();
    double t1 = st_points[i].t();
    double t2 = st_points[i].t();
    if (std::fabs(t1 - t2) <= FLAGS_lattice_epsilon) {
      continue;
    }
    double jerk = (dds2 - dds1) / (t2 - t1);
    double cost = jerk / FLAGS_longitudinal_jerk_upper_bound;
    cost_sqr_sum += cost * cost;
    cost_abs_sum += std::fabs(cost);
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::LonObjectiveCost(
    const PtrTrajectory1d& lon_trajectory,
    const PlanningTarget& planning_target,
    const std::vector<double>& ref_s_dots) const {
  double t_max = lon_trajectory->ParamLength();
  double dist_s =
      lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);

  double speed_cost_sqr_sum = 0.0;
  double speed_cost_weight_sum = 0.0;
  for (std::size_t i = 0; i < ref_s_dots.size(); ++i) {
    double t = i * FLAGS_trajectory_time_resolution;
    double cost = ref_s_dots[i] - lon_trajectory->Evaluate(1, t);
    speed_cost_sqr_sum += t * t * std::fabs(cost);
    speed_cost_weight_sum += t * t;
  }
  double speed_cost =
      speed_cost_sqr_sum / (speed_cost_weight_sum + FLAGS_lattice_epsilon);
  double dist_travelled_cost = 1.0 / (1.0 + dist_s);
  return (speed_cost * FLAGS_weight_target_speed +
          dist_travelled_cost * FLAGS_weight_dist_travelled) /
         (FLAGS_weight_target_speed + FLAGS_weight_dist_travelled);
}

double TrajectoryEvaluator::LonObjectiveCost(
    const std::vector<SpeedPoint>& st_points,
    const PlanningTarget& planning_target,
    const std::vector<double>& ref_s_dots) const {
  if (st_points.size() < 1) {
    return 0.0;
  }
  double dist_s = st_points[st_points.size() - 1].s() - st_points[0].s();
  double speed_cost_sqr_sum = 0.0;
  double speed_cost_weight_sum = 0.0;
  std::size_t point_idx = 0;
  for (std::size_t i = 0; i < ref_s_dots.size(); ++i) {
    double t = i * FLAGS_trajectory_time_resolution;
    while (point_idx + 1 < st_points.size()) {
      if (st_points[point_idx + 1].t() <= t) {
        ++point_idx;
      }
    }
    double speed = st_points.back().v();
    if (point_idx + 1 < st_points.size()) {
      speed = 0.5 * (st_points[point_idx].v() + st_points[point_idx + 1].v());
    }
    double cost = ref_s_dots[i] - speed;
    speed_cost_sqr_sum += t * t * std::fabs(cost);
    speed_cost_weight_sum += t * t;
  }
  double speed_cost =
      speed_cost_sqr_sum / (speed_cost_weight_sum + FLAGS_lattice_epsilon);
  double dist_travelled_cost = 1.0 / (1.0 + dist_s);
  return (speed_cost * FLAGS_weight_target_speed +
          dist_travelled_cost * FLAGS_weight_dist_travelled) /
         (FLAGS_weight_target_speed + FLAGS_weight_dist_travelled);
}

// TODO(all): consider putting pointer of reference_line_info and frame
// while constructing trajectory evaluator
double TrajectoryEvaluator::LonCollisionCost(
    const PtrTrajectory1d& lon_trajectory) const {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (std::size_t i = 0; i < path_time_intervals_.size(); ++i) {
    const auto& pt_interval = path_time_intervals_[i];
    if (pt_interval.empty()) {
      continue;
    }
    double t = i * FLAGS_trajectory_time_resolution;
    double traj_s = lon_trajectory->Evaluate(0, t);
    double sigma = FLAGS_lon_collision_cost_std;
    for (const auto& m : pt_interval) {
      double dist = 0.0;
      if (traj_s < m.first - FLAGS_lon_collision_yield_buffer) {
        dist = m.first - FLAGS_lon_collision_yield_buffer - traj_s;
      } else if (traj_s > m.second + FLAGS_lon_collision_overtake_buffer) {
        dist = traj_s - m.second - FLAGS_lon_collision_overtake_buffer;
      }
      double cost = std::exp(-dist * dist / (2.0 * sigma * sigma));

      cost_sqr_sum += cost * cost;
      cost_abs_sum += cost;
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::LonCollisionCost(
    const std::vector<common::SpeedPoint>& st_points) const {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (std::size_t i = 0; i < path_time_intervals_.size(); ++i) {
    const auto& pt_interval = path_time_intervals_[i];
    if (pt_interval.empty()) {
      continue;
    }
    double t = i * FLAGS_trajectory_time_resolution;
    double traj_s = std::numeric_limits<double>::infinity();
    if (!InterpolateDenseStPoints(st_points, t, &traj_s)) {
      AERROR << "AutoTuning LonCollisionCost InterpolateDenseStPoints Error";
      return traj_s;
    }
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
      cost_abs_sum += std::fabs(cost);
    }
  }
  return cost_sqr_sum / (cost_abs_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::CentripetalAccelerationCost(
    const PtrTrajectory1d& lon_trajectory) const {

  // Assumes the vehicle is not obviously deviate from the reference line.
  double centripetal_acc_sum = 0.0;
  double centripetal_acc_sqr_sum = 0.0;
  for (double t = 0.0; t < FLAGS_trajectory_time_length;
       t += FLAGS_trajectory_time_resolution) {
    double s = lon_trajectory->Evaluate(0, t);
    double v = lon_trajectory->Evaluate(1, t);
    PathPoint ref_point = PathMatcher::MatchToPath(*reference_line_, s);
    CHECK(ref_point.has_kappa());
    double centripetal_acc = v * v * ref_point.kappa();
    centripetal_acc_sum += std::fabs(centripetal_acc);
    centripetal_acc_sqr_sum += centripetal_acc * centripetal_acc;
  }

  return centripetal_acc_sqr_sum /
         (centripetal_acc_sum + FLAGS_lattice_epsilon);
}

double TrajectoryEvaluator::CentripetalAccelerationCost(
    const std::vector<SpeedPoint>& st_points) const {
  double centripetal_acc_sum = 0.0;
  double centripetal_acc_sqr_sum = 0.0;
  for (const auto& st_point : st_points) {
    double s = st_point.s();
    double v = st_point.v();
    PathPoint ref_point = PathMatcher::MatchToPath(*reference_line_, s);
    CHECK(ref_point.has_kappa());
    double centripetal_acc = v * v * ref_point.kappa();
    centripetal_acc_sum += std::fabs(centripetal_acc);
    centripetal_acc_sqr_sum += centripetal_acc * centripetal_acc;
  }

  return centripetal_acc_sqr_sum /
         (centripetal_acc_sum + FLAGS_lattice_epsilon);
}

std::vector<double> TrajectoryEvaluator::ComputeLongitudinalGuideVelocity(
    const PlanningTarget& planning_target) const {
  std::vector<double> reference_s_dot;

  double brake_a = FLAGS_longitudinal_acceleration_lower_bound *
                   FLAGS_comfort_acceleration_factor;

  double cruise_v = planning_target.cruise_speed();

  if (!planning_target.has_stop_point()) {
    ConstantAccelerationTrajectory1d lon_traj(init_s_[0], cruise_v);
    lon_traj.AppendSegment(0.0, FLAGS_trajectory_time_length);

    for (double t = 0.0; t < FLAGS_trajectory_time_length;
         t += FLAGS_trajectory_time_resolution) {
      reference_s_dot.emplace_back(lon_traj.Evaluate(1, t));
    }
  } else {
    double dist_s = planning_target.stop_point().s() - init_s_[0];
    if (dist_s < FLAGS_lattice_epsilon) {
      ConstantAccelerationTrajectory1d lon_traj(init_s_[0], 0.0);
      lon_traj.AppendSegment(0.0, FLAGS_trajectory_time_length);

      for (double t = 0.0; t < FLAGS_trajectory_time_length;
           t += FLAGS_trajectory_time_resolution) {
        reference_s_dot.emplace_back(lon_traj.Evaluate(1, t));
      }
      return reference_s_dot;
    }

    double brake_s = -cruise_v * cruise_v * 0.5 / brake_a;
    // need more distance to brake
    if (brake_s > dist_s) {
      double desired_v = std::sqrt(-2.0 * brake_a * dist_s);
      double brake_t = -desired_v / brake_a;
      ConstantAccelerationTrajectory1d lon_traj(init_s_[0], desired_v);
      lon_traj.AppendSegment(brake_a, brake_t);

      if (lon_traj.ParamLength() < FLAGS_trajectory_time_length) {
        lon_traj.AppendSegment(
            0.0, FLAGS_trajectory_time_length - lon_traj.ParamLength());
      }
      for (double t = 0.0; t < FLAGS_trajectory_time_length;
           t += FLAGS_trajectory_time_resolution) {
        reference_s_dot.emplace_back(lon_traj.Evaluate(1, t));
      }
    } else {
      double brake_t = -cruise_v / brake_a;
      double cruise_s = dist_s - brake_s;
      double cruise_t = cruise_s / cruise_v;

      ConstantAccelerationTrajectory1d lon_traj(init_s_[0], cruise_v);
      lon_traj.AppendSegment(0.0, cruise_t);
      lon_traj.AppendSegment(brake_a, brake_t);
      if (lon_traj.ParamLength() < FLAGS_trajectory_time_length) {
        lon_traj.AppendSegment(
            0.0, FLAGS_trajectory_time_length - lon_traj.ParamLength());
      }
      for (double t = 0.0; t < FLAGS_trajectory_time_length;
           t += FLAGS_trajectory_time_resolution) {
        reference_s_dot.emplace_back(lon_traj.Evaluate(1, t));
      }
    }
  }
  return reference_s_dot;
}

bool TrajectoryEvaluator::InterpolateDenseStPoints(
    const std::vector<SpeedPoint>& st_points, double t, double* traj_s) const {
  CHECK_GT(st_points.size(), 1);
  if (t < st_points[0].t() || t > st_points[st_points.size() - 1].t()) {
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

}  // namespace planning
}  // namespace apollo
