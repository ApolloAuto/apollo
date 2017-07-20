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
 * @file dp_poly_path_config.cpp
 **/

#include "modules/planning/optimizer/dp_poly_path/dp_poly_path_config.h"

namespace apollo {
namespace planning {

DpPolyPathConfig::DpPolyPathConfig(const boost::property_tree::ptree &_ptree) {
  // TODO:need to add;
}

void DpPolyPathConfig::set_sample_level(const size_t sample_level) {
  _sample_level = sample_level;
}

void DpPolyPathConfig::set_sample_points_num_each_level(const size_t sample_points_num_each_level) {
  _sample_points_num_each_level = sample_points_num_each_level;
}

void DpPolyPathConfig::set_step_length_max(const double step_length_max) {
  _step_length_max = step_length_max;
}

void DpPolyPathConfig::set_step_length_min(const double step_length_min) {
  _step_length_min = step_length_min;
}

void DpPolyPathConfig::set_lateral_sample_offset(const double lateral_sample_offset) {
  _lateral_sample_offset = lateral_sample_offset;
}

void DpPolyPathConfig::set_lateral_adjust_coeff(const double lateral_adjust_coeff) {
  _lateral_adjust_coeff = lateral_adjust_coeff;
}

void DpPolyPathConfig::set_eval_time_interval(const double eval_time_interval) {
  _eval_time_interval = eval_time_interval;
}

size_t DpPolyPathConfig::sample_level() const {
  return _sample_level;
}

size_t DpPolyPathConfig::sample_points_num_each_level() const {
  return _sample_points_num_each_level;
}

double DpPolyPathConfig::step_length_max() const {
  return _step_length_max;
}

double DpPolyPathConfig::step_length_min() const {
  return _step_length_min;
}

double DpPolyPathConfig::lateral_sample_offset() const {
  return _lateral_sample_offset;
}

double DpPolyPathConfig::lateral_adjust_coeff() const {
  return _lateral_adjust_coeff;
}

double DpPolyPathConfig::eval_time_interval() const {
  return _eval_time_interval;
}

} // namespace planning
} // namespace apollo
