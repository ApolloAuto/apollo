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
 * @file qp_spline_path_constraint.h
 **/

#ifndef BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_CONFIGURATION_H_
#define BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_CONFIGURATION_H_

#include "boost/property_tree/ptree.hpp"

namespace apollo {
namespace planning {

class QPSplinePathConfiguration {
 public:
  QPSplinePathConfiguration() = default;
  QPSplinePathConfiguration(const boost::property_tree::ptree& property);

  // spline configuration
  void set_spline_order(const std::size_t spline_order);
  void set_number_of_spline(const std::size_t num_of_spline);

  // constraint configuration
  void set_number_of_fx_constraint_knots(const std::size_t num_constraint);

  // kernel configuration
  void set_regularization_weight(const double weight);
  void set_derivative_weight(const double weight);
  void set_second_derivative_weight(const double weight);
  void set_third_derivative_weight(const double weight);
  void set_reference_line_weight(const double weight);
  void set_num_refline_point(const std::size_t num_refline_point);
  void set_num_output(const std::size_t num_output);
  // spline configuration
  std::size_t spline_order() const;
  std::size_t number_of_spline() const;

  // constraint configuration
  std::size_t number_of_fx_constraint_knots() const;
  double time_resolution() const;

  // kernel configuration
  double regularization_weight() const;
  double derivative_weight() const;
  double second_derivative_weight() const;
  double third_derivative_weight() const;
  double reference_line_weight() const;
  std::size_t num_refline_point() const;

  // output configuration
  std::size_t num_output() const;

 private:
  std::size_t _spline_order = 6;

  std::size_t _number_of_spline = 5;

  std::size_t _number_of_fx_constraint_knots = 6;

  double _time_resolution = 0.1;

  double _regularization_weight = 0.001;

  double _derivative_weight = 0.0;

  double _second_derivative_weight = 0.0;

  double _third_derivative_weight = 100.0;

  double _reference_line_weight = 0.0;

  std::size_t _num_refline_point = 10;

  std::size_t _num_output = 100;
};
}  // namespace planning
}  // namespace apollo

#endif  // BAIDU_IDG_HOUSTON_OPTIMIZER_QP_SPLINE_PATH_OPTIMIZER_QP_SPLINE_PATH_CONFIGURATION_H_
