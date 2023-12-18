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

#include "modules/planning/planning_base/math/curve1d/quintic_polynomial_curve1d.h"

#include "gtest/gtest.h"

#include "modules/planning/planning_base/math/curve1d/quartic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

TEST(QuinticPolynomialCurve1dTest, basic_test) {
  double x0 = 0.0;
  double dx0 = 1.0;
  double ddx0 = 0.8;

  double x1 = 10.0;
  double dx1 = 5.0;
  double ddx1 = 0.0;

  double t = 8.0;

  QuinticPolynomialCurve1d curve(x0, dx0, ddx0, x1, dx1, ddx1, t);
  auto e_x0 = curve.Evaluate(0, 0.0);
  auto e_dx0 = curve.Evaluate(1, 0.0);
  auto e_ddx0 = curve.Evaluate(2, 0.0);

  auto e_x1 = curve.Evaluate(0, t);
  auto e_dx1 = curve.Evaluate(1, t);
  auto e_ddx1 = curve.Evaluate(2, t);

  auto e_t = curve.ParamLength();

  EXPECT_NEAR(x0, e_x0, 1.0e-6);
  EXPECT_NEAR(dx0, e_dx0, 1.0e-6);
  EXPECT_NEAR(ddx0, e_ddx0, 1.0e-6);

  EXPECT_NEAR(x1, e_x1, 1.0e-6);
  EXPECT_NEAR(dx1, e_dx1, 1.0e-6);
  EXPECT_NEAR(ddx1, e_ddx1, 1.0e-6);

  EXPECT_NEAR(t, e_t, 1.0e-6);
}

TEST(QuinticPolynomialCurve1dTest, IntegratedFromQuarticCurve) {
  QuarticPolynomialCurve1d quartic_curve(2, 1, 4, 3, 2, 4);
  QuinticPolynomialCurve1d quintic_curve;
  quintic_curve.IntegratedFromQuarticCurve(quartic_curve, 1);
  for (double value = 0.0; value < 4.1; value += 0.1) {
    EXPECT_NEAR(quartic_curve.Evaluate(0, value),
                quintic_curve.Evaluate(1, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(1, value),
                quintic_curve.Evaluate(2, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(2, value),
                quintic_curve.Evaluate(3, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(3, value),
                quintic_curve.Evaluate(4, value), 1e-8);
  }
}
}  // namespace planning
}  // namespace apollo
