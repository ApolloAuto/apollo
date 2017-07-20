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
 * @file : sl_analytic_transformation.cc
 **/

#include "modules/planning/math/sl_analytic_transformation.h"

#include <cmath>

#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

double SLAnalyticTransformation::calculate_theta(const double theta,
                                                 const double kappa_ref,
                                                 const double l,
                                                 const double dl) {
  return theta + std::atan2(dl, 1 - l * kappa_ref);
}

double SLAnalyticTransformation::calculate_kappa(const double kappa_ref,
                                                 const double dkappa_ref,
                                                 const double l,
                                                 const double dl,
                                                 const double ddl) {
  double denominator = (dl * dl + (1 - l * kappa_ref) * (1 - l * kappa_ref));
  if (Double::compare(denominator, 0.0, 1e-8) == 0) {
    return 0.0;
  }
  denominator = std::pow(denominator, 1.5);
  const double numerator = kappa_ref + ddl - 2 * l * kappa_ref * kappa_ref -
                           l * ddl * kappa_ref +
                           l * l * kappa_ref * kappa_ref * kappa_ref +
                           l * dl * dkappa_ref + 2 * dl * dl * kappa_ref;
  return numerator / denominator;
}

Eigen::Vector2d SLAnalyticTransformation::calculate_xypoint(
    const double theta, const Eigen::Vector2d& point_ref, const double l) {
  double x = point_ref.x() - l * std::sin(theta);
  double y = point_ref.y() + l * std::cos(theta);
  Eigen::Vector2d result{x, y};
  return result;
}

double SLAnalyticTransformation::calculate_lateral_derivative(
    const double theta_ref, const double theta, const double l,
    const double kappa_ref) {
  return (1 - kappa_ref * l) * std::tan(theta - theta_ref);
}

double SLAnalyticTransformation::calculate_second_order_lateral_derivative(
    const double theta_ref, const double theta, const double kappa_ref,
    const double kappa, const double dkappa_ref, const double l) {
  const double dl =
      calculate_lateral_derivative(theta_ref, theta, l, kappa_ref);
  const double theta_diff = theta - theta_ref;
  const double cos_theta_diff = std::cos(theta_diff);
  // TODO(fanhaoyang): add sanity check for invalid input
  return -(dkappa_ref * l + kappa_ref * dl) * std::tan(theta - theta_ref) +
         (1 - kappa_ref * l) / (cos_theta_diff * cos_theta_diff) *
             (kappa * (1 - kappa_ref * l) / cos_theta_diff - kappa_ref);
}
}  // namespace planning
}  // namespace apollo
