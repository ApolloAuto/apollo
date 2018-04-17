/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/math/cartesian_frenet_conversion.h"

#include <array>
#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(TestCartesianFrenetConversion, cartesian_to_frenet_test) {
  double rs = 10.0;
  double rx = 0.0;
  double ry = 0.0;
  double rtheta = M_PI / 4.0;
  double rkappa = 0.1;
  double rdkappa = 0.01;
  double x = -1.0;
  double y = 1.0;
  double v = 2.0;
  double a = 0.0;
  double theta = M_PI / 3.0;
  double kappa = 0.11;

  std::array<double, 3> s_conditions;
  std::array<double, 3> d_conditions;

  CartesianFrenetConverter::cartesian_to_frenet(
      rs, rx, ry, rtheta, rkappa, rdkappa, x, y, v, a, theta, kappa,
      &s_conditions, &d_conditions);

  double x_out;
  double y_out;
  double theta_out;
  double kappa_out;
  double v_out;
  double a_out;

  CartesianFrenetConverter::frenet_to_cartesian(
      rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x_out,
      &y_out, &theta_out, &kappa_out, &v_out, &a_out);

  EXPECT_NEAR(x, x_out, 1.0e-6);
  EXPECT_NEAR(y, y_out, 1.0e-6);
  EXPECT_NEAR(theta, theta_out, 1.0e-6);
  EXPECT_NEAR(kappa, kappa_out, 1.0e-6);
  EXPECT_NEAR(v, v_out, 1.0e-6);
  EXPECT_NEAR(a, a_out, 1.0e-6);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
