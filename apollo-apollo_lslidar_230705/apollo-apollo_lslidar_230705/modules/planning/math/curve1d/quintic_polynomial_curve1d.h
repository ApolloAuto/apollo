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
 * @file quintic_polynomial_curve1d.h
 **/

#pragma once

#include <array>
#include <string>

#include "modules/planning/math/curve1d/polynomial_curve1d.h"

namespace apollo {
namespace planning {

// 1D quintic polynomial curve:
// (x0, dx0, ddx0) -- [0, param] --> (x1, dx1, ddx1)
class QuinticPolynomialCurve1d : public PolynomialCurve1d {
 public:
  QuinticPolynomialCurve1d() = default;

  QuinticPolynomialCurve1d(const std::array<double, 3>& start,
                           const std::array<double, 3>& end,
                           const double param);

  QuinticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1, const double ddx1,
                           const double param);

  QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d& other);

  void SetParam(const double x0, const double dx0, const double ddx0,
                const double x1, const double dx1, const double ddx1,
                const double param);

  void IntegratedFromQuarticCurve(const PolynomialCurve1d& other,
                                  const double init_value);

  virtual ~QuinticPolynomialCurve1d() = default;

  double Evaluate(const std::uint32_t order, const double p) const override;

  double ParamLength() const override { return param_; }
  std::string ToString() const override;

  double Coef(const size_t order) const override;

  size_t Order() const override { return 5; }

 protected:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1, const double ddx1,
                           const double param);

  // f = sum(coef_[i] * x^i), i from 0 to 5
  std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};
  std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
};

}  // namespace planning
}  // namespace apollo
