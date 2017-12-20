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

#ifndef MODULES_PLANNING_LATTICE_UTIL_LATTICE_QUARTIC_POLYNOMIAL_CURVE1D_H_
#define MODULES_PLANNING_LATTICE_UTIL_LATTICE_QUARTIC_POLYNOMIAL_CURVE1D_H_

#include <array>

#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

class LatticeQuarticPolynomialCurve1d : public QuarticPolynomialCurve1d {
 public:
  LatticeQuarticPolynomialCurve1d() = default;

  LatticeQuarticPolynomialCurve1d(
      const std::array<double, 3>& start,
      const std::array<double, 2>& end,
      const double param);

  LatticeQuarticPolynomialCurve1d(
      const double x0, const double dx0, const double ddx0,
      const double dx1, const double ddx1,
      const double param);

  // LatticeQuarticPolynomialCurve1d(const QuarticPolynomialCurve1d& other);

  virtual ~LatticeQuarticPolynomialCurve1d() = default;

  void SetSampleCondition(const double s, const double ds, const double dds,
                          const double t);

  std::array<double, 3> GetSampledSCondition();

  double GetSampledTime();

 private:
  double sampled_s_;
  double sampled_ds_;
  double sampled_dds_;
  double sampled_t_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_LATTICE_UTIL_LATTICE_QUARTIC_POLYNOMIAL_CURVE1D_H_
