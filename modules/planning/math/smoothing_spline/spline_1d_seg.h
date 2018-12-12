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
 * @file : spline_1d_seg.h
 * @brief: polynomial smoothing spline
 **/

#pragma once

#include <vector>

#include "Eigen/Core"

#include "modules/planning/math/polynomial_xd.h"

namespace apollo {
namespace planning {

class Spline1dSeg {
 public:
  // order represents the highest order.
  explicit Spline1dSeg(const uint32_t order);
  explicit Spline1dSeg(const std::vector<double>& params);
  ~Spline1dSeg() = default;

  void SetParams(const std::vector<double>& params);
  double operator()(const double x) const;
  double Derivative(const double x) const;
  double SecondOrderDerivative(const double x) const;
  double ThirdOrderDerivative(const double x) const;

  const PolynomialXd& spline_func() const;
  const PolynomialXd& Derivative() const;
  const PolynomialXd& SecondOrderDerivative() const;
  const PolynomialXd& ThirdOrderDerivative() const;

 private:
  inline void SetSplineFunc(const PolynomialXd& spline_func);

  PolynomialXd spline_func_;
  PolynomialXd derivative_;
  PolynomialXd second_order_derivative_;
  PolynomialXd third_order_derivative_;
};

}  // namespace planning
}  // namespace apollo
