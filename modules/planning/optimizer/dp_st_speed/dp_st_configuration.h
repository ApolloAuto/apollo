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

#ifndef MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_CONFIGURATION_H_
#define MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_CONFIGURATION_H_

#include <cinttypes>
#include <cstddef>
#include <limits>

namespace apollo {
namespace planning {

class DpStConfiguration {
 public:
  DpStConfiguration() = default;

  double total_path_length() const;
  double total_time() const;

  std::uint32_t matrix_dimension_s() const;
  std::uint32_t matrix_dimension_t() const;

  double speed_weight() const;
  double accel_weight() const;
  double jerk_weight() const;

  double obstacle_weight() const;
  double reference_weight() const;

  double st_graph_default_point_cost() const;
  double obstacle_cost_factor() const;

  double default_speed_cost() const;
  double exceed_speed_penalty() const;
  double low_speed_penalty() const;

  double accel_penalty() const;
  double decel_penalty() const;

  double positive_jerk_coeff() const;
  double negative_jerk_coeff() const;

  double max_speed() const;
  double max_acceleration() const;
  double max_deceleration() const;

  double go_down_buffer() const;
  double go_up_buffer() const;

  void set_total_path_length(const double s);

 private:
  double _total_path_length = 80.0;
  double _total_time = 8.0;

  std::uint32_t _matrix_dimension_s = 100;
  std::uint32_t _matrix_dimension_t = 10;

  double _speed_weight = 0.0;
  double _accel_weight = 10.0;
  double _jerk_weight = 10.0;
  double _obstacle_weight = 1.0;
  double _reference_weight = 0.0;

  double _go_down_buffer = 5.0;
  double _go_up_buffer = 5.0;

  // obstacle cost config
  double _st_graph_default_point_cost = 1e10;
  double _obstacle_cost_factor = -300;

  // speed cost config
  double _default_speed_cost = 1.0;
  double _exceed_speed_penalty = 10.0;
  double _low_speed_penalty = 2.5;

  // accel cost config
  double _accel_penalty = 2.0;
  double _decel_penalty = 2.0;

  // jerk cost config
  double _positive_jerk_coeff = 1.0;
  double _negative_jerk_coeff = 300;

  // other constraint
  double _max_speed = 20;
  double _max_acceleration = 4.5;
  double _max_deceleration = -4.5;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_ST_SPEED_DP_ST_CONFIGURATION_H_
