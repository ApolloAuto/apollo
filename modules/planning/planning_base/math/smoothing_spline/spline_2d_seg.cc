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
 * @file : spline_2d_seg.cc
 * @brief: polynomial smoothing spline
 **/

#include "modules/planning/planning_base/math/smoothing_spline/spline_2d_seg.h"

namespace apollo {
namespace planning {
Spline2dSeg::Spline2dSeg(const uint32_t order)
    : spline_func_x_(order), spline_func_y_(order) {
  derivative_x_ = PolynomialXd::DerivedFrom(spline_func_x_);
  derivative_y_ = PolynomialXd::DerivedFrom(spline_func_y_);
  second_derivative_x_ = PolynomialXd::DerivedFrom(derivative_x_);
  second_derivative_y_ = PolynomialXd::DerivedFrom(derivative_y_);
  third_derivative_x_ = PolynomialXd::DerivedFrom(second_derivative_x_);
  third_derivative_y_ = PolynomialXd::DerivedFrom(second_derivative_y_);
}

Spline2dSeg::Spline2dSeg(const std::vector<double>& x_param,
                         const std::vector<double>& y_param)
    : spline_func_x_(x_param), spline_func_y_(y_param) {
  derivative_x_ = PolynomialXd::DerivedFrom(spline_func_x_);
  derivative_y_ = PolynomialXd::DerivedFrom(spline_func_y_);
  second_derivative_x_ = PolynomialXd::DerivedFrom(derivative_x_);
  second_derivative_y_ = PolynomialXd::DerivedFrom(derivative_y_);
  third_derivative_x_ = PolynomialXd::DerivedFrom(second_derivative_x_);
  third_derivative_y_ = PolynomialXd::DerivedFrom(second_derivative_y_);
}

bool Spline2dSeg::SetParams(const std::vector<double>& x_param,
                            const std::vector<double>& y_param) {
  if (x_param.size() != y_param.size()) {
    return false;
  }

  spline_func_x_ = PolynomialXd(x_param);
  spline_func_y_ = PolynomialXd(y_param);
  derivative_x_ = PolynomialXd::DerivedFrom(spline_func_x_);
  derivative_y_ = PolynomialXd::DerivedFrom(spline_func_y_);
  second_derivative_x_ = PolynomialXd::DerivedFrom(derivative_x_);
  second_derivative_y_ = PolynomialXd::DerivedFrom(derivative_y_);
  third_derivative_x_ = PolynomialXd::DerivedFrom(second_derivative_x_);
  third_derivative_y_ = PolynomialXd::DerivedFrom(second_derivative_y_);
  return true;
}

std::pair<double, double> Spline2dSeg::operator()(const double t) const {
  return std::make_pair(spline_func_x_(t), spline_func_y_(t));
}

double Spline2dSeg::x(const double t) const { return spline_func_x_(t); }

double Spline2dSeg::y(const double t) const { return spline_func_y_(t); }

double Spline2dSeg::DerivativeX(const double t) const {
  return derivative_x_(t);
}

double Spline2dSeg::DerivativeY(const double t) const {
  return derivative_y_(t);
}

double Spline2dSeg::SecondDerivativeX(const double t) const {
  return second_derivative_x_(t);
}

double Spline2dSeg::SecondDerivativeY(const double t) const {
  return second_derivative_y_(t);
}

double Spline2dSeg::ThirdDerivativeX(const double t) const {
  return third_derivative_x_(t);
}

double Spline2dSeg::ThirdDerivativeY(const double t) const {
  return third_derivative_y_(t);
}

const PolynomialXd& Spline2dSeg::spline_func_x() const {
  return spline_func_x_;
}

const PolynomialXd& Spline2dSeg::spline_func_y() const {
  return spline_func_y_;
}

const PolynomialXd& Spline2dSeg::DerivativeX() const { return derivative_x_; }

const PolynomialXd& Spline2dSeg::DerivativeY() const { return derivative_y_; }

const PolynomialXd& Spline2dSeg::SecondDerivativeX() const {
  return second_derivative_x_;
}

const PolynomialXd& Spline2dSeg::SecondDerivativeY() const {
  return second_derivative_y_;
}

const PolynomialXd& Spline2dSeg::ThirdDerivativeX() const {
  return third_derivative_x_;
}

const PolynomialXd& Spline2dSeg::ThirdDerivativeY() const {
  return third_derivative_y_;
}

}  // namespace planning
}  // namespace apollo
