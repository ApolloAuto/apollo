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

#include "modules/planning/common/data_center.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

DpStCost::DpStCost(const DpStSpeedConfig& dp_st_configuration)
    : _dp_st_configuration(dp_st_configuration) {
  _unit_s = _dp_st_configuration.total_path_length() /
            _dp_st_configuration.matrix_dimension_s();
  _unit_t = _dp_st_configuration.total_time() /
            _dp_st_configuration.matrix_dimension_t();
}

// TODO(all): normalize cost with time
double DpStCost::obstacle_cost(
    const STPoint& point,
    const std::vector<StGraphBoundary>& obs_boundary) const {
  double total_cost = 0.0;
  for (const StGraphBoundary& boundary : obs_boundary) {
    if (point.s() < 0 || boundary.IsPointInBoundary(point)) {
      total_cost += _dp_st_configuration.st_graph_default_point_cost();
    } else {
      ::apollo::common::math::Vec2d vec2d = {point.t(), point.s()};
      double distance = boundary.DistanceTo(vec2d);
      total_cost += _dp_st_configuration.st_graph_default_point_cost() *
                    std::exp(_dp_st_configuration.obstacle_cost_factor() /
                             boundary.characteristic_length() * distance);
    }
  }
  return total_cost;
}

double DpStCost::reference_cost(const STPoint& point,
                                const STPoint& reference_point) const {
  return _dp_st_configuration.reference_weight() *
         (point.s() - reference_point.s()) * (point.s() - reference_point.s()) *
         _unit_t;
}

double DpStCost::speed_cost(const STPoint& first, const STPoint& second,
                            const double speed_limit) const {
  double cost = 0.0;
  double speed = (second.s() - first.s()) / _unit_t;
  if (Double::compare(speed, 0.0) < 0) {
    cost = _dp_st_configuration.st_graph_default_point_cost();
  }
  double det_speed = (speed - speed_limit) / speed_limit;
  if (Double::compare(det_speed, 0.0) > 0) {
    cost = _dp_st_configuration.exceed_speed_penalty() *
           _dp_st_configuration.default_speed_cost() * fabs(speed * speed) *
           _unit_t;
  } else if (Double::compare(det_speed, 0.0) < 0) {
    cost = _dp_st_configuration.low_speed_penalty() *
           _dp_st_configuration.default_speed_cost() * -det_speed * _unit_t;
  } else {
    cost = 0.0;
  }
  return cost;
}

double DpStCost::accel_cost(const double accel) const {
  double accel_sq = accel * accel;
  double max_acc = _dp_st_configuration.max_acceleration();
  double max_dec = _dp_st_configuration.max_deceleration();
  double accel_penalty = _dp_st_configuration.accel_penalty();
  double decel_penalty = _dp_st_configuration.decel_penalty();
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

double DpStCost::accel_cost_by_three_points(const STPoint& first,
                                            const STPoint& second,
                                            const STPoint& third) const {
  double accel = (first.s() + third.s() - 2 * second.s()) / (_unit_t * _unit_t);
  return accel_cost(accel);
}

double DpStCost::accel_cost_by_two_points(const double pre_speed,
                                          const STPoint& pre_point,
                                          const STPoint& curr_point) const {
  double current_speed = (curr_point.s() - pre_point.s()) / _unit_t;
  double accel = (current_speed - pre_speed) / _unit_t;
  return accel_cost(accel);
}

double DpStCost::jerk_cost(const double jerk) const {
  double jerk_sq = jerk * jerk;
  double cost = 0.0;
  if (Double::compare(jerk, 0.0) > 0) {
    cost = _dp_st_configuration.positive_jerk_coeff() * jerk_sq * _unit_t;
  } else if (Double::compare(jerk, 0.0) < 0) {
    cost = _dp_st_configuration.negative_jerk_coeff() * jerk_sq * _unit_t;
  } else {
    cost = 0.0;
  }
  return cost;
}

double DpStCost::jerk_cost_by_four_points(const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third,
                                          const STPoint& forth) const {
  double jerk = (first.s() - 3 * second.s() + 3 * third.s() - forth.s()) /
                (_unit_t * _unit_t * _unit_t);
  return jerk_cost(jerk);
}

double DpStCost::jerk_cost_by_two_points(const double pre_speed,
                                         const double pre_acc,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) const {
  double curr_speed = (curr_point.s() - pre_point.s()) / _unit_t;
  double curr_accel = (curr_speed - pre_speed) / _unit_t;
  double jerk = (curr_accel - pre_acc) / _unit_t;
  return jerk_cost(jerk);
}

double DpStCost::jerk_cost_by_three_points(const double first_speed,
                                           const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) const {
  double pre_speed = (second.s() - first.s()) / _unit_t;
  double pre_acc = (pre_speed - first_speed) / _unit_t;
  double curr_speed = (third.s() - second.s()) / _unit_t;
  double curr_acc = (curr_speed - pre_speed) / _unit_t;
  double jerk = (curr_acc - pre_acc) / _unit_t;
  return jerk_cost(jerk);
}

}  // namespace planning
}  // namespace apollo
