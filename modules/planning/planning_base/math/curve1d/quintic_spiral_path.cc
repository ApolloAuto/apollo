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
 * @file quintic_spiral_path.cpp
 **/

#include "modules/planning/planning_base/math/curve1d/quintic_spiral_path.h"

namespace apollo {
namespace planning {

QuinticSpiralPath::QuinticSpiralPath(const double x0, const double dx0,
                                     const double ddx0, const double x1,
                                     const double dx1, const double ddx1,
                                     const double p)
    : QuinticPolynomialCurve1d(x0, dx0, ddx0, x1, dx1, ddx1, p) {
  ACHECK(p > 0.0);

  double p2 = p * p;
  double p3 = p2 * p;
  double p4 = p3 * p;
  double p5 = p2 * p3;
  double p6 = p3 * p3;

  // derive a
  // double a = -6.0 * x0 / p5 - 3.0 * dx0 / p4 - 0.5 * ddx0 / p3 + 6.0 * x1 /
  // p5 - 3.0 * dx1 / p4 + 0.5 * ddx1 / p3;
  coef_deriv_[5][0] = -6.0 / p5;

  coef_deriv_[5][1] = -3.0 / p4;

  coef_deriv_[5][2] = -0.5 / p3;

  coef_deriv_[5][3] = 6.0 / p5;

  coef_deriv_[5][4] = -3.0 / p4;

  coef_deriv_[5][5] = 0.5 / p3;

  coef_deriv_[5][6] = 30.0 * x0 / p6 + 12.0 * dx0 / p5 + 1.5 * ddx0 / p4 -
                      30.0 * x1 / p6 + 12.0 * dx1 / p5 - 1.5 * ddx1 / p4;

  // derive b
  // double b = 15.0 * x0 / p4 + 8.0 * dx0 / p3 + 1.5 * ddx0 / p2 - 15.0 * x1 /
  // p4 + 7.0 * dx1 / p3 - ddx1 / p2;
  coef_deriv_[4][0] = 15.0 / p4;

  coef_deriv_[4][1] = 8.0 / p3;

  coef_deriv_[4][2] = 1.5 / p2;

  coef_deriv_[4][3] = -15.0 / p4;

  coef_deriv_[4][4] = 7.0 / p3;

  coef_deriv_[4][5] = -1.0 / p2;

  coef_deriv_[4][6] = -60.0 * x0 / p5 - 24.0 * dx0 / p4 - 3.0 * ddx0 / p3 +
                      60.0 * x1 / p5 - 21.0 * dx1 / p4 + 2.0 * ddx1 / p3;

  // derive c
  // double c = -10.0 * x0 / p3 - 6.0 * dx0 / p2 - 1.5 * ddx0 / p + 10.0 * x1 /
  // p3 - 4.0 * dx1 / p2 + 0.5 * ddx1 / p;
  coef_deriv_[3][0] = -10.0 / p3;

  coef_deriv_[3][1] = -6.0 / p2;

  coef_deriv_[3][2] = -1.5 / p;

  coef_deriv_[3][3] = 10.0 / p3;

  coef_deriv_[3][4] = -4.0 / p2;

  coef_deriv_[3][5] = 0.5 / p;

  coef_deriv_[3][6] = 30.0 * x0 / p4 + 12.0 * dx0 / p3 + 1.5 * ddx0 / p2 -
                      30.0 * x1 / p4 + 8.0 * dx1 / p3 - 0.5 * ddx1 / p2;

  // derive d
  // double d = 0.5 * ddx0;
  coef_deriv_[2][0] = 0.0;

  coef_deriv_[2][1] = 0.0;

  coef_deriv_[2][2] = 0.5;

  coef_deriv_[2][3] = 0.0;

  coef_deriv_[2][4] = 0.0;

  coef_deriv_[2][5] = 0.0;

  coef_deriv_[2][6] = 0.0;

  // derive e
  // double e = dx0;
  coef_deriv_[1][0] = 0.0;

  coef_deriv_[1][1] = 1.0;

  coef_deriv_[1][2] = 0.0;

  coef_deriv_[1][3] = 0.0;

  coef_deriv_[1][4] = 0.0;

  coef_deriv_[1][5] = 0.0;

  coef_deriv_[1][6] = 0.0;

  // derive f
  // double f = x0;
  coef_deriv_[0][0] = 1.0;

  coef_deriv_[0][1] = 0.0;

  coef_deriv_[0][2] = 0.0;

  coef_deriv_[0][3] = 0.0;

  coef_deriv_[0][4] = 0.0;

  coef_deriv_[0][5] = 0.0;

  coef_deriv_[0][6] = 0.0;
}

QuinticSpiralPath::QuinticSpiralPath(const std::array<double, 3>& start,
                                     const std::array<double, 3>& end,
                                     const double delta_s)
    : QuinticSpiralPath(start[0], start[1], start[2], end[0], end[1], end[2],
                        delta_s) {}

double QuinticSpiralPath::DeriveTheta(const size_t param_index,
                                      const double r) const {
  double s = param_ * r;
  double s2 = s * s;
  double s3 = s2 * s;
  double s4 = s2 * s2;
  double s5 = s3 * s2;

  double derivative =
      coef_deriv_[5][param_index] * s5 + coef_deriv_[4][param_index] * s4 +
      coef_deriv_[3][param_index] * s3 + coef_deriv_[2][param_index] * s2 +
      coef_deriv_[1][param_index] * s + coef_deriv_[0][param_index];

  if (param_index == DELTA_S) {
    derivative += coef_[5] * 5.0 * s4 * r + coef_[4] * 4.0 * s3 * r +
                  coef_[3] * 3.0 * s2 * r + coef_[2] * 2.0 * s * r +
                  coef_[1] * r;
  }
  return derivative;
}

double QuinticSpiralPath::DeriveKappaDerivative(const size_t param_index,
                                                const double r) const {
  double s = param_ * r;
  double s2 = s * s;
  double s3 = s2 * s;
  double s4 = s2 * s2;

  double derivative = 5.0 * coef_deriv_[5][param_index] * s4 +
                      4.0 * coef_deriv_[4][param_index] * s3 +
                      3.0 * coef_deriv_[3][param_index] * s2 +
                      2.0 * coef_deriv_[2][param_index] * s +
                      coef_deriv_[1][param_index];

  if (param_index == DELTA_S) {
    derivative += 5.0 * coef_[5] * 4.0 * s3 * r +
                  4.0 * coef_[4] * 3.0 * s2 * r + 3.0 * coef_[3] * 2.0 * s * r +
                  2.0 * coef_[2] * r;
  }
  return derivative;
}

double QuinticSpiralPath::DeriveDKappaDerivative(const size_t param_index,
                                                 const double r) const {
  double s = param_ * r;
  double s2 = s * s;
  double s3 = s2 * s;

  double derivative = 20.0 * coef_deriv_[5][param_index] * s3 +
                      12.0 * coef_deriv_[4][param_index] * s2 +
                      6.0 * coef_deriv_[3][param_index] * s +
                      2.0 * coef_deriv_[2][param_index];

  if (param_index == DELTA_S) {
    derivative += 20.0 * coef_[5] * 3.0 * s2 * r +
                  12.0 * coef_[4] * 2.0 * s * r + 6.0 * coef_[3] * r;
  }
  return derivative;
}

double QuinticSpiralPath::DeriveD2KappaDerivative(const size_t param_index,
                                                  const double r) const {
  double s = param_ * r;
  double s2 = s * s;

  double derivative = 60.0 * coef_deriv_[5][param_index] * s2 +
                      24.0 * coef_deriv_[4][param_index] * s +
                      6.0 * coef_deriv_[3][param_index];

  if (param_index == DELTA_S) {
    derivative += 60.0 * coef_[5] * 2.0 * s * r + 24.0 * coef_[4] * r;
  }
  return derivative;
}

}  // namespace planning
}  // namespace apollo
