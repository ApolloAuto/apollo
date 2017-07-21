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
 * @file dp_st_configuration.h
 **/

#include "optimizer/dp_st_speed_optimizer/dp_st_configuration.h"

namespace apollo {
namespace planning {

double DpStConfiguration::total_path_length() const {
  return _total_path_length;
}

double DpStConfiguration::total_time() const { return _total_time; }

std::size_t DpStConfiguration::matrix_dimension_s() const {
  return _matrix_dimension_s;
}

std::size_t DpStConfiguration::matrix_dimension_t() const {
  return _matrix_dimension_t;
}

double DpStConfiguration::speed_weight() const { return _speed_weight; }

double DpStConfiguration::accel_weight() const { return _accel_weight; }

double DpStConfiguration::jerk_weight() const { return _jerk_weight; }

double DpStConfiguration::obstacle_weight() const { return _obstacle_weight; }

double DpStConfiguration::reference_weight() const { return _reference_weight; }

double DpStConfiguration::st_graph_default_point_cost() const {
  return _st_graph_default_point_cost;
}

double DpStConfiguration::obstacle_cost_factor() const {
  return _obstacle_cost_factor;
}

double DpStConfiguration::default_speed_cost() const {
  return _default_speed_cost;
}

double DpStConfiguration::exceed_speed_penalty() const {
  return _exceed_speed_penalty;
}

double DpStConfiguration::low_speed_penalty() const {
  return _low_speed_penalty;
}

double DpStConfiguration::accel_penalty() const { return _accel_penalty; }

double DpStConfiguration::decel_penalty() const { return _decel_penalty; }

double DpStConfiguration::positive_jerk_coeff() const {
  return _positive_jerk_coeff;
}

double DpStConfiguration::negative_jerk_coeff() const {
  return _negative_jerk_coeff;
}

double DpStConfiguration::max_speed() const { return _max_speed; }

double DpStConfiguration::max_acceleration() const { return _max_acceleration; }

double DpStConfiguration::max_deceleration() const { return _max_deceleration; }

double DpStConfiguration::go_down_buffer() const { return _go_down_buffer; }

double DpStConfiguration::go_up_buffer() const { return _go_up_buffer; }

void DpStConfiguration::set_total_path_length(const double s) {
  _total_path_length = s;
}
}  // namespace planning
}  // namespace apollo
