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
 * @file dp_st_cost.cc
 **/

#include "modules/planning/tasks/dp_st_speed/dp_st_cost.h"

#include <limits>

#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {

DpStCost::DpStCost(const DpStSpeedConfig& dp_st_speed_config)
    : dp_st_speed_config_(dp_st_speed_config),
      unit_s_(dp_st_speed_config_.total_path_length() /
              dp_st_speed_config_.matrix_dimension_s()),
      unit_t_(dp_st_speed_config_.total_time() /
              dp_st_speed_config_.matrix_dimension_t()) {
  unit_v_ = unit_s_ / unit_t_;
}

// TODO(all): normalize cost with time
double DpStCost::GetObstacleCost(
    const StGraphPoint& st_graph_point,
    const std::vector<const StBoundary*>& st_boundaries) const {
  double total_cost = 0.0;
  constexpr double inf = std::numeric_limits<double>::infinity();
  const double unit_v = unit_s_ / unit_t_;
  const auto& st_point = st_graph_point.point();
  if (st_point.s() < 0) {
    return inf;
  }
  for (const StBoundary* boundary : st_boundaries) {
    if (boundary->IsPointInBoundary(st_point)) {
      if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
        total_cost += unit_v * ((st_graph_point.index_s() + 1.0) /
                                (st_graph_point.index_t() + 1.0)) *
                      dp_st_speed_config_.keep_clear_cost_factor();
      } else {
        return inf;
      }
    } else {
      const double distance = boundary->DistanceS(st_point);
      total_cost += dp_st_speed_config_.default_obstacle_cost() *
                    std::exp(dp_st_speed_config_.obstacle_cost_factor() /
                             boundary->characteristic_length() * distance);
    }
  }
  return total_cost * unit_t_;
}

double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const {
  return dp_st_speed_config_.reference_weight() *
         (point.s() - reference_point.s()) * (point.s() - reference_point.s()) *
         unit_t_;
}

double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit) const {
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0) {
    return std::numeric_limits<double>::infinity();
  }
  double det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0) {
    cost = dp_st_speed_config_.exceed_speed_penalty() *
           dp_st_speed_config_.default_speed_cost() * fabs(speed * speed) *
           unit_t_;
  } else if (det_speed < 0) {
    cost = dp_st_speed_config_.low_speed_penalty() *
           dp_st_speed_config_.default_speed_cost() * -det_speed * unit_t_;
  } else {
    cost = 0.0;
  }
  return cost;
}

double DpStCost::GetAccelCost(const double accel) const {
  const double accel_sq = accel * accel;
  double max_acc = dp_st_speed_config_.max_acceleration();
  double max_dec = dp_st_speed_config_.max_deceleration();
  double accel_penalty = dp_st_speed_config_.accel_penalty();
  double decel_penalty = dp_st_speed_config_.decel_penalty();
  double cost = 0.0;
  if (accel > 0.0) {
    cost = accel_penalty * accel_sq;
  } else {
    cost = decel_penalty * accel_sq;
  }
  cost += accel_sq * decel_penalty * decel_penalty /
              (1 + std::exp(1.0 * (accel - max_dec))) +
          accel_sq * accel_penalty * accel_penalty /
              (1 + std::exp(-1.0 * (accel - max_acc)));
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) const {
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) const {
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk) const {
  double jerk_sq = jerk * jerk;
  double cost = 0.0;
  if (jerk > 0) {
    cost = dp_st_speed_config_.positive_jerk_coeff() * jerk_sq * unit_t_;
  } else {
    cost = dp_st_speed_config_.negative_jerk_coeff() * jerk_sq * unit_t_;
  }
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) const {
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) const {
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) const {
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
