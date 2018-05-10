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
 * @file : spline_1d.h
 * @brief: piecewise smoothing spline class
 *       1. Model description: piecewise smoothing spline are made by pieces of
 *smoothing splines
 *          joint at knots;
 *       2. To guarantee smoothness, pieces value at knots should joint together
 *with
 *           same value, derivative, and etc. Higher the order, More smoothness
 *the piecewise spline;
 **/

#ifndef MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_H_
#define MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_H_

#include <vector>

#include "Eigen/Core"

#include "modules/planning/math/polynomial_xd.h"
#include "modules/planning/math/smoothing_spline/affine_constraint.h"
#include "modules/planning/math/smoothing_spline/spline_1d_seg.h"

namespace apollo {
namespace planning {

class Spline1d {
 public:
  Spline1d(const std::vector<double>& x_knots, const uint32_t order);

  // @brief: given x return f(x) value, derivative, second order derivative and
  // the third order;
  double operator()(const double x) const;
  double Derivative(const double x) const;
  double SecondOrderDerivative(const double x) const;
  double ThirdOrderDerivative(const double x) const;

  // @brief: set spline segments
  bool SetSplineSegs(const Eigen::MatrixXd& param_matrix, const uint32_t order);

  const std::vector<double>& x_knots() const;
  uint32_t spline_order() const;

 private:
  uint32_t FindIndex(const double x) const;

 private:
  std::vector<Spline1dSeg> splines_;
  std::vector<double> x_knots_;
  uint32_t spline_order_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SMOOTHING_SPLINE_SPLINE_1D_H_
