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
 * @file qp_spline_path_configuration.cpp
 **/
#include "optimizer/qp_spline_path_optimizer/qp_spline_path_configuration.h"

namespace apollo {
namespace planning {

QPSplinePathConfiguration::QPSplinePathConfiguration(
    const boost::property_tree::ptree& property) {
  _spline_order = property.get<std::size_t>("spline_order");
  _number_of_spline = property.get<std::size_t>("number_of_spline");
  _number_of_fx_constraint_knots =
      property.get<std::size_t>("number_of_fx_constraint_knots");
  _time_resolution = property.get<double>("time_resolution");
  _regularization_weight = property.get<double>("regularization_weight");
  _derivative_weight = property.get<double>("derivative_weight");
  _second_derivative_weight = property.get<double>("second_derivative_weight");
  _third_derivative_weight = property.get<double>("third_derivative_weight");
  _reference_line_weight = property.get<double>("reference_line_weight");
  _num_refline_point = property.get<std::size_t>("num_refline_point");
  _num_output = property.get<std::size_t>("num_output");
}

void QPSplinePathConfiguration::set_spline_order(
    const std::size_t spline_order) {
  _spline_order = spline_order;
}

void QPSplinePathConfiguration::set_number_of_spline(
    const std::size_t num_of_spline) {
  _number_of_spline = num_of_spline;
}

// constraint configuration
void QPSplinePathConfiguration::set_number_of_fx_constraint_knots(
    const std::size_t num_constraint) {
  _number_of_fx_constraint_knots = num_constraint;
}

double QPSplinePathConfiguration::time_resolution() const {
  return _time_resolution;
}

// kernel configuration
void QPSplinePathConfiguration::set_regularization_weight(const double weight) {
  _regularization_weight = weight;
}

void QPSplinePathConfiguration::set_derivative_weight(const double weight) {
  _derivative_weight = weight;
}

void QPSplinePathConfiguration::set_second_derivative_weight(
    const double weight) {
  _second_derivative_weight = weight;
}

void QPSplinePathConfiguration::set_third_derivative_weight(
    const double weight) {
  _third_derivative_weight = weight;
}

void QPSplinePathConfiguration::set_reference_line_weight(const double weight) {
  _reference_line_weight = weight;
}

void QPSplinePathConfiguration::set_num_refline_point(
    const std::size_t num_refline_point) {
  _num_refline_point = num_refline_point;
}

// spline configuration
std::size_t QPSplinePathConfiguration::spline_order() const {
  return _spline_order;
}

std::size_t QPSplinePathConfiguration::number_of_spline() const {
  return _number_of_spline;
}

// constraint configuration
std::size_t QPSplinePathConfiguration::number_of_fx_constraint_knots() const {
  return _number_of_fx_constraint_knots;
}

// kernel configuration
double QPSplinePathConfiguration::regularization_weight() const {
  return _regularization_weight;
}

double QPSplinePathConfiguration::derivative_weight() const {
  return _derivative_weight;
}

double QPSplinePathConfiguration::second_derivative_weight() const {
  return _second_derivative_weight;
}

double QPSplinePathConfiguration::third_derivative_weight() const {
  return _third_derivative_weight;
}

double QPSplinePathConfiguration::reference_line_weight() const {
  return _reference_line_weight;
}

std::size_t QPSplinePathConfiguration::num_refline_point() const {
  return _num_refline_point;
}

// output configuration
std::size_t QPSplinePathConfiguration::num_output() const {
  return _num_output;
}
}  // namespace planning
}  // namespace apollo