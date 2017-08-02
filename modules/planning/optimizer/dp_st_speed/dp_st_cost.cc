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

#include "modules/planning/optimizer/dp_st_speed/dp_st_cost.h"

#include <limits>

#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

DpStCost::DpStCost(const DpStSpeedConfig& dp_st_speed_config)
    : _dp_st_speed_config(dp_st_speed_config) {
  _unit_s = _dp_st_speed_config.total_path_length() /
            _dp_st_speed_config.matrix_dimension_s();
  _unit_t = _dp_st_speed_config.total_time() /
            _dp_st_speed_config.matrix_dimension_t();
}

// TODO(all): normalize cost with time
double DpStCost::GetObstacleCost(
    const STPoint& point,
    const std::vector<StGraphBoundary>& st_graph_boundaries) const {
  double total_cost = 0.0;
  for (const StGraphBoundary& boundary : st_graph_boundaries) {
    if (point.s() < 0 || boundary.IsPointInBoundary(point)) {
      total_cost += _dp_st_speed_config.st_graph_default_point_cost();
    } else {
      double distance = boundary.DistanceTo(point);
      total_cost += _dp_st_speed_config.st_graph_default_point_cost() *
                    std::exp(_dp_st_speed_config.obstacle_cost_factor() /
                             boundary.characteristic_length() * distance);
    }
  }
  return total_cost;
}

double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const {
  return _dp_st_speed_config.reference_weight() *
         (point.s() - reference_point.s()) * (point.s() - reference_point.s()) *
         _unit_t;
}

double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit) const {
  double cost = 0.0;
  double speed = (second.s() - first.s()) / _unit_t;
  if (Double::compare(speed, 0.0) < 0) {
    cost = _dp_st_speed_config.st_graph_default_point_cost();
  }
  double det_speed = (speed - speed_limit) / speed_limit;
  if (Double::compare(det_speed, 0.0) > 0) {
    cost = _dp_st_speed_config.exceed_speed_penalty() *
           _dp_st_speed_config.default_speed_cost() * fabs(speed * speed) *
           _unit_t;
  } else if (Double::compare(det_speed, 0.0) < 0) {
    cost = _dp_st_speed_config.low_speed_penalty() *
           _dp_st_speed_config.default_speed_cost() * -det_speed * _unit_t;
  } else {
    cost = 0.0;
  }
  return cost;
}

double DpStCost::GetAccelCost(const double accel) const {
  double accel_sq = accel * accel;
  double max_acc = _dp_st_speed_config.max_acceleration();
  double max_dec = _dp_st_speed_config.max_deceleration();
  double accel_penalty = _dp_st_speed_config.accel_penalty();
  double decel_penalty = _dp_st_speed_config.decel_penalty();
  double cost = 0.0;
  if (accel > 0.0) {
    cost = accel_penalty * accel_sq * _unit_t;
  } else {
    cost = decel_penalty * accel_sq * _unit_t;
  }
  cost += accel_sq * decel_penalty * decel_penalty /
              (1 + std::exp(1.0 * (accel - max_dec))) +
          accel_sq * accel_penalty * accel_penalty /
              (1 + std::exp(-1.0 * (accel - max_acc)));
  return cost;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) const {
  double accel = (first.s() + third.s() - 2 * second.s()) / (_unit_t * _unit_t);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) const {
  double current_speed = (curr_point.s() - pre_point.s()) / _unit_t;
  double accel = (current_speed - pre_speed) / _unit_t;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk) const {
  double jerk_sq = jerk * jerk;
  double cost = 0.0;
  if (Double::compare(jerk, 0.0) > 0) {
    cost = _dp_st_speed_config.positive_jerk_coeff() * jerk_sq * _unit_t;
  } else if (Double::compare(jerk, 0.0) < 0) {
    cost = _dp_st_speed_config.negative_jerk_coeff() * jerk_sq * _unit_t;
  } else {
    cost = 0.0;
  }
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& forth) const {
  double jerk = (first.s() - 3 * second.s() + 3 * third.s() - forth.s()) /
                (_unit_t * _unit_t * _unit_t);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) const {
  double curr_speed = (curr_point.s() - pre_point.s()) / _unit_t;
  double curr_accel = (curr_speed - pre_speed) / _unit_t;
  double jerk = (curr_accel - pre_acc) / _unit_t;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) const {
  double pre_speed = (second.s() - first.s()) / _unit_t;
  double pre_acc = (pre_speed - first_speed) / _unit_t;
  double curr_speed = (third.s() - second.s()) / _unit_t;
  double curr_acc = (curr_speed - pre_speed) / _unit_t;
  double jerk = (curr_acc - pre_acc) / _unit_t;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
