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
 * @file
 **/

#pragma once

#include <array>
#include <string>

#include "modules/planning/math/curve1d/polynomial_curve1d.h"

namespace apollo {
namespace planning {

class CubicPolynomialCurve1d : public PolynomialCurve1d {
 public:
  CubicPolynomialCurve1d() = default;
  virtual ~CubicPolynomialCurve1d() = default;

  CubicPolynomialCurve1d(const std::array<double, 3>& start, const double end,
                         const double param);

  /**
   * x0 is the value when f(x = 0);
   * dx0 is the value when f'(x = 0);
   * ddx0 is the value when f''(x = 0);
   * f(x = param) = x1
   */
  CubicPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                         const double x1, const double param);

  void DerivedFromQuarticCurve(const PolynomialCurve1d& other);

  double Evaluate(const std::uint32_t order, const double p) const override;

  double ParamLength() const override { return param_; }
  std::string ToString() const override;

  double Coef(const size_t order) const override;

  size_t Order() const override { return 3; }

 private:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double param);
  std::array<double, 4> coef_ = {{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
  double end_condition_ = 0.0;
};

}  // namespace planning
}  // namespace apollo
