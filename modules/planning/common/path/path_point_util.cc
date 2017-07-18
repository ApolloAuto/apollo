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
 * @file path_point_util.cc
 **/

#include "modules/planning/common/path/path_point_util.h"

#include <utility>

#include "modules/planning/math/hermite_spline.h"
#include "modules/planning/math/integration.h"

namespace apollo {
namespace planning {
namespace util {

common::PathPoint interpolate(const common::PathPoint& p0,
                              const common::PathPoint& p1, const double s) {
  double s0 = p0.s();
  double s1 = p1.s();
  CHECK(s0 <= s && s <= s1);

  std::array<double, 2> gx0{{p0.theta(), p0.kappa()}};
  std::array<double, 2> gx1{{p1.theta(), p1.kappa()}};
  HermiteSpline<double, 3> geometry_spline(gx0, gx1, s0, s1);
  auto func_cos_theta = [&geometry_spline](const double s) {
    auto theta = geometry_spline.evaluate(0, s);
    return std::cos(theta);
  };
  auto func_sin_theta = [&geometry_spline](const double s) {
    auto theta = geometry_spline.evaluate(0, s);
    return std::sin(theta);
  };

  double x = p0.x() + Integration::gauss_legendre(func_cos_theta, s0, s);
  double y = p0.y() + Integration::gauss_legendre(func_sin_theta, s0, s);
  double theta = geometry_spline.evaluate(0, s);
  double kappa = geometry_spline.evaluate(1, s);
  double dkappa = geometry_spline.evaluate(2, s);
  double d2kappa = geometry_spline.evaluate(3, s);

  common::PathPoint p;
  p.set_x(x);
  p.set_y(y);
  p.set_theta(theta);
  p.set_kappa(kappa);
  p.set_dkappa(dkappa);
  p.set_ddkappa(d2kappa);
  p.set_s(s);
  return std::move(p);
}

common::PathPoint interpolate_linear_approximation(
    const common::PathPoint& left, const common::PathPoint& right,
    const double s) {
  double s0 = left.s();
  double s1 = right.s();
  CHECK(s0 < s1);

  common::PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * left.x() + weight * right.x();
  double y = (1 - weight) * left.y() + weight * right.y();
  double cos_heading =
      (1 - weight) * std::cos(left.theta()) + weight * std::cos(right.theta());
  double sin_heading =
      (1 - weight) * std::sin(left.theta()) + weight * std::sin(right.theta());
  double theta = std::atan2(sin_heading, cos_heading);
  double kappa = (1 - weight) * left.kappa() + weight * right.kappa();
  double dkappa = (1 - weight) * left.dkappa() + weight * right.dkappa();
  double ddkappa = (1 - weight) * left.ddkappa() + weight * right.ddkappa();
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  return path_point;
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
