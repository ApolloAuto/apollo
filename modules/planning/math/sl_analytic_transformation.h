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
 * @file : sl_analytic_transformation.h
 * @brief: explicit analytic form of sl to x, y, theta, kappa transformation
 *
 *           x = x_ref - l * sin(theta)
 *           y = y_ref + l * cos(theta)
 *           theta = theta_ref + atan2(dl / ds, 1 - l * kappa_ref);
 *           To derive kappa, we need the formula below:
 *           kappa = (x' * y'' - y' * x'') / ((x')^2 + (y')^2)^(3/2)
 *           after some calculations, kappa is a expilicit form of theta, l and
 *its derivatives
 * @Notice: Theta's expicit form need first derivative of l and theta
 *            Kappa's explicit form need second derivative of l and theta
 **/

#ifndef MODULES_PLANNING_MATH_SL_ANALYTIC_TRANSFORMATION_H_
#define MODULES_PLANNING_MATH_SL_ANALYTIC_TRANSFORMATION_H_

#include <cstddef>

#include "modules/common/math/vec2d.h"

namespace apollo {
namespace planning {

class SLAnalyticTransformation {
 public:
  // given sl point extract x, y, theta, kappa
  static double CalculateTheta(const double theta_ref, const double kappa_ref,
                               const double l, const double dl);
  static double CalculateKappa(const double kappa_ref, const double dkappa_ref,
                                const double l, const double dl,
                                const double ddl);
  static common::math::Vec2d CalculateXYPoint(
      const double theta_ref, const common::math::Vec2d& point_ref,
      const double l);
  /**
   * @brief: given sl, theta, and road's theta, kappa, extract derivative l,
   *second order derivative l:
   * @reference:  in paper: optimal trajectory generation for dynamic street
   * scenarios in a frenet frame
   **/
  static double CalculateLateralDerivative(const double theta_ref,
                                           const double theta, const double l,
                                           const double kappa_ref);

  // given sl, theta, and road's theta, kappa, extract second order derivative
  static double CalculateSecondOrderLateralDerivative(
      const double theta_ref, const double theta, const double kappa_ref,
      const double kappa, const double dkappa_ref, const double l);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SL_ANALYTIC_TRANSFORMATION_H_
