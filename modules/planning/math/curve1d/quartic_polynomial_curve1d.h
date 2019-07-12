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
 * @file quartic_polynomial_curve1d.h
 **/

#pragma once

#include <array>
#include <string>

#include "modules/planning/math/curve1d/polynomial_curve1d.h"

namespace apollo {
namespace planning {

// 1D quartic polynomial curve: (x0, dx0, ddx0) -- [0, param] --> (dx1, ddx1)
class QuarticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  QuarticPolynomialCurve1d() = default;

  QuarticPolynomialCurve1d(const std::array<double, 3>& start,
                           const std::array<double, 2>& end,
                           const double param);

  QuarticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                           const double dx1, const double ddx1,
                           const double param);

  QuarticPolynomialCurve1d(const QuarticPolynomialCurve1d& other);

  virtual ~QuarticPolynomialCurve1d() = default;

  double Evaluate(const std::uint32_t order, const double p) const override;

  /**
   * Interface with refine quartic polynomial by meets end first order
   * and start second order boundary condition:
   * @param  x0    init point x location
   * @param  dx0   init point derivative
   * @param  ddx0  init point second order derivative
   * @param  x1    end point x location
   * @param  dx1   end point derivative
   * @param  param parameter length
   * @return       self
   */
  QuarticPolynomialCurve1d& FitWithEndPointFirstOrder(
      const double x0, const double dx0, const double ddx0, const double x1,
      const double dx1, const double param);

  /**
   * Interface with refine quartic polynomial by meets end point second order
   * and start point first order boundary condition
   */
  QuarticPolynomialCurve1d& FitWithEndPointSecondOrder(
      const double x0, const double dx0, const double x1, const double dx1,
      const double ddx1, const double param);

  /*
   * Integrated from cubic curve with init value
   */
  QuarticPolynomialCurve1d& IntegratedFromCubicCurve(
      const PolynomialCurve1d& other, const double init_value);

  /*
   * Derived from quintic curve
   */
  QuarticPolynomialCurve1d& DerivedFromQuinticCurve(
      const PolynomialCurve1d& other);

  double ParamLength() const override { return param_; }

  std::string ToString() const override;

  double Coef(const size_t order) const override;

  size_t Order() const override { return 4; }

 private:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double dx1, const double ddx1,
                           const double param);

  std::array<double, 5> coef_ = {{0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
  std::array<double, 2> end_condition_ = {{0.0, 0.0}};
};

}  // namespace planning
}  // namespace apollo
