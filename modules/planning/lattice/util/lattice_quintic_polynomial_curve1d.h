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

#ifndef MODULES_PLANNING_LATTICE_UTIL_LATTICE_QUINTIC_POLYNOMIAL_CURVE1D_H_
#define MODULES_PLANNING_LATTICE_UTIL_LATTICE_QUINTIC_POLYNOMIAL_CURVE1D_H_

#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

class LatticeQuinticPolynomialCurve1d : public QuinticPolynomialCurve1d {
 public:
  LatticeQuinticPolynomialCurve1d() = default;

  LatticeQuinticPolynomialCurve1d(
      const std::array<double, 3>& start,
      const std::array<double, 3>& end,
      const double param);

  LatticeQuinticPolynomialCurve1d(
      const double x0, const double dx0, const double ddx0,
      const double x1, const double dx1, const double ddx1,
      const double param);

  // LatticeQuinticPolynomialCurve1d(const QuarticPolynomialCurve1d& other);

  virtual ~LatticeQuinticPolynomialCurve1d() = default;

 private:
  double sampled_s_;
  double sampled_ds_;
  double sampled_dds_;
  double sampled_t_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_UTIL_LATTICE_QUINTIC_POLYNOMIAL_CURVE1D_H_
