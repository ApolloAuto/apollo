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

#include "modules/planning/lattice/util/lattice_quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

LatticeQuinticPolynomialCurve1d::LatticeQuinticPolynomialCurve1d(
    const std::array<double, 3>& start, const std::array<double, 3>& end,
    const double param)
  : QuinticPolynomialCurve1d(start, end, param) {}

LatticeQuinticPolynomialCurve1d::LatticeQuinticPolynomialCurve1d(
    const double x0, const double dx0, const double ddx0,
    const double x1, const double dx1, const double ddx1,
    const double param)
  : QuinticPolynomialCurve1d(x0, dx0, ddx0, x1, dx1, ddx1, param) {}

void LatticeQuinticPolynomialCurve1d::SetSampleCondition(
    const double s, const double ds, const double dds, const double t) {
  sampled_s_ = s;
  sampled_ds_ = ds;
  sampled_dds_ = ds;
  sampled_t_ = t;
}

std::array<double, 3>
LatticeQuinticPolynomialCurve1d::GetSampledSCondition() {
  return {sampled_s_, sampled_ds_, sampled_dds_};
}

double LatticeQuinticPolynomialCurve1d::GetSampledTime() {
  return sampled_t_;
}

}  // namespace planning
}  // namespace apollo
