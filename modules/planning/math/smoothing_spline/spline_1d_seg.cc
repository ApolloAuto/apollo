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
 * @file : spline_1d_seg.cc
 * @brief: polynomial smoothing spline segment
 **/

#include "modules/planning/math/smoothing_spline/spline_1d_seg.h"

namespace apollo {
namespace planning {

Spline1dSeg::Spline1dSeg(const uint32_t order) {
  SetSplineFunc(PolynomialXd(order));
}

Spline1dSeg::Spline1dSeg(const std::vector<double>& params) {
  SetSplineFunc(PolynomialXd(params));
}

void Spline1dSeg::SetParams(const std::vector<double>& params) {
  SetSplineFunc(PolynomialXd(params));
}

void Spline1dSeg::SetSplineFunc(const PolynomialXd& spline_func) {
  spline_func_ = spline_func;
  derivative_ = PolynomialXd::DerivedFrom(spline_func_);
  second_order_derivative_ = PolynomialXd::DerivedFrom(derivative_);
  third_order_derivative_ = PolynomialXd::DerivedFrom(second_order_derivative_);
}

double Spline1dSeg::operator()(const double x) const { return spline_func_(x); }

double Spline1dSeg::Derivative(const double x) const { return derivative_(x); }

double Spline1dSeg::SecondOrderDerivative(const double x) const {
  return second_order_derivative_(x);
}

double Spline1dSeg::ThirdOrderDerivative(const double x) const {
  return third_order_derivative_(x);
}

const PolynomialXd& Spline1dSeg::spline_func() const { return spline_func_; }

const PolynomialXd& Spline1dSeg::Derivative() const { return derivative_; }

const PolynomialXd& Spline1dSeg::SecondOrderDerivative() const {
  return second_order_derivative_;
}

const PolynomialXd& Spline1dSeg::ThirdOrderDerivative() const {
  return third_order_derivative_;
}

}  // namespace planning
}  // namespace apollo
