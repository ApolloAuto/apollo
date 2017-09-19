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
 * @file curvature.cc
 **/
#include "modules/planning/math/curve_math.h"

#include <cmath>

namespace apollo {
namespace planning {

double CurveMath::ComputeCurvature(const double dx, const double d2x,
                                   const double dy, const double d2y) {
  double a = dx * d2y - dy * d2x;
  double dx_dy_norm_square = dx * dx + dy * dy;
  double dx_dy_norm = std::sqrt(dx_dy_norm_square);
  double b = dx_dy_norm_square * dx_dy_norm;
  return a / b;
}

double CurveMath::ComputeCurvatureDerivative(const double dx, const double d2x,
                                             const double d3x, const double dy,
                                             const double d2y,
                                             const double d3y) {
  double a = dx * d2y - dy * d2x;
  double dx_dy_norm_square = dx * dx + dy * dy;
  double dx_dy_norm = std::sqrt(dx_dy_norm_square);
  double b = dx_dy_norm_square * dx_dy_norm;

  double a_dot = dx * d3y - dy * d3x;
  double b_dot = 3.0 * dx_dy_norm * (dx * d2x + dy * d2y);

  double dkappa =
      (a_dot * b - a * b_dot) /
      (dx_dy_norm_square * dx_dy_norm_square * dx_dy_norm_square * dx_dy_norm);
  return dkappa;
}

}  // namespace planning
}  // namespace apollo
