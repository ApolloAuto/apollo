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

#include "modules/planning/tasks/dp_st_speed/dp_st_cost.h"

#include <limits>

#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

DpStCost::DpStCost(const DpStSpeedConfig& config,
                   const std::vector<const PathObstacle*>& obstacles,
                   const common::TrajectoryPoint& init_point)
    : config_(config),
      obstacles_(obstacles),
      init_point_(init_point),
      unit_s_(config_.total_path_length() / config_.matrix_dimension_s()),
      unit_t_(config_.total_time() / config_.matrix_dimension_t()),
      unit_v_(unit_s_ / unit_t_) {}

double DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) {
  const double s = st_graph_point.point().s();
  const double t = st_graph_point.point().t();

  double cost = 0.0;
  for (const auto* obstacle : obstacles_) {
    auto boundary = obstacle->st_boundary();
    const double kIgnoreDistance = 200.0;
    if (boundary.min_s() > kIgnoreDistance) {
      continue;
    }
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }
    if (obstacle->IsBlockingObstacle() &&
        boundary.IsPointInBoundary(STPoint(s, t))) {
      return kInf;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;

    const std::string key =
        boundary.id() + "#" + std::to_string(st_graph_point.index_t());
    if (boundary_range_map_.find(key) == boundary_range_map_.end()) {
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_range_map_[key] = std::make_pair(s_upper, s_lower);
    } else {
      s_upper = boundary_range_map_[key].first;
      s_lower = boundary_range_map_[key].second;
    }
    if (s < s_lower) {
      constexpr double kSafeTimeBuffer = 3.0;
      const double len = obstacle->obstacle()->Speed() * kSafeTimeBuffer;
      if (s + len < s_lower) {
        continue;
      } else {
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                std::pow((len - s_lower + s), 2);
      }
    } else if (s > s_upper) {
      const double kSafeDistance = 20.0;  // or calculated from velocity
      if (s > s_upper + kSafeDistance) {
        continue;
      } else {
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                std::pow((kSafeDistance + s_upper - s), 2);
      }
    }
  }
  return cost * unit_t_;
}

double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const {
  return config_.reference_weight() * (point.s() - reference_point.s()) *
         (point.s() - reference_point.s()) * unit_t_;
}

double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit) const {
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0) {
    return kInf;
  }
  double det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0) {
    cost = config_.exceed_speed_penalty() * config_.default_speed_cost() *
           fabs(speed * speed) * unit_t_;
  } else if (det_speed < 0) {
    cost = config_.low_speed_penalty() * config_.default_speed_cost() *
           -det_speed * unit_t_;
  } else {
    cost = 0.0;
  }
  return cost;
}

double DpStCost::GetAccelCost(const double accel) {
  double cost = 0.0;
  constexpr double kEpsilon = 0.1;
  const int accel_key = static_cast<int>(accel / kEpsilon + 0.5);
  if (accel_cost_map_.find(accel_key) == accel_cost_map_.end()) {
    const double accel_sq = accel * accel;
    double max_acc = config_.max_acceleration();
    double max_dec = config_.max_deceleration();
    double accel_penalty = config_.accel_penalty();
    double decel_penalty = config_.decel_penalty();

    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_map_[accel_key] = cost;
  } else {
    cost = accel_cost_map_[accel_key];
  }
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) {
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) {
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk) {
  double cost = 0.0;
  constexpr double kEpsilon = 0.1;
  const int jerk_key = static_cast<int>(jerk / kEpsilon + 0.5);
  if (jerk_cost_map_.find(jerk_key) == jerk_cost_map_.end()) {
    double jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_map_[jerk_key] = cost;
  } else {
    cost = jerk_cost_map_[jerk_key];
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) {
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) {
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
