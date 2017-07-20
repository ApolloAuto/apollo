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
 * @file dp_poly_path_config.h
 **/

#ifndef MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_DP_POLY_PATH_CONFIG_H
#define MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_DP_POLY_PATH_CONFIG_H

#include "boost/property_tree/ptree.hpp"

namespace apollo {
namespace planning {

class DpPolyPathConfig {
 public:
  DpPolyPathConfig() = default;
  DpPolyPathConfig(const boost::property_tree::ptree &_ptree);
  void set_sample_level(const size_t sample_level);
  void set_sample_points_num_each_level(const size_t sample_points_num_each_level);
  void set_step_length_max(const double step_length_max);
  void set_step_length_min(const double step_length_min);
  void set_lateral_sample_offset(const double lateral_sample_offset);
  void set_lateral_adjust_coeff(const double lateral_adjust_coeff);
  void set_eval_time_interval(const double eval_time_interval);

  size_t sample_level() const;
  size_t sample_points_num_each_level() const;
  double step_length_max() const;
  double step_length_min() const;
  double lateral_sample_offset() const;
  double lateral_adjust_coeff() const;
  double eval_time_interval() const;

 private:
  // Sampler Config
  size_t _sample_level = 8;
  size_t _sample_points_num_each_level = 9;
  double _step_length_max = 10.0;
  double _step_length_min = 5.0;
  double _lateral_sample_offset = 0.5;
  double _lateral_adjust_coeff = 0.5;
  // Trajectory Cost Config
  double _eval_time_interval = 0.1;

};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_OPTIMIZER_DP_POLY_PATH_DP_POLY_PATH_CONFIG_H
