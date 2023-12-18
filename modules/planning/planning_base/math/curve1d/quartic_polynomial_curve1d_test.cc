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

/**
 * @file
 **/

#include "modules/planning/planning_base/math/curve1d/quartic_polynomial_curve1d.h"

#include "gtest/gtest.h"

#include "modules/planning/planning_base/math/curve1d/cubic_polynomial_curve1d.h"
#include "modules/planning/planning_base/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

TEST(QuarticPolynomialCurve1dTest, Evaluate) {
  {
    double x0 = 0.0;
    double dx0 = 0.0;
    double ddx0 = 0.0;
    double dx1 = 10.0;
    double ddx1 = 1.0;
    double param = 8.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(2, 0.0), 1e-8);
  }
  {
    double x0 = 0.0;
    double dx0 = 0.0;
    double ddx0 = 0.0;
    double dx1 = 5.0;
    double ddx1 = 1.0;
    double param = 3.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(2, 0.0), 1e-8);
  }

  {
    double x0 = 1.0;
    double dx0 = 2.0;
    double ddx0 = 3.0;
    double dx1 = 5.0;
    double ddx1 = 1.0;
    double param = 3.0;

    QuarticPolynomialCurve1d curve(x0, dx0, ddx0, dx1, ddx1, param);
    EXPECT_NEAR(dx1, curve.Evaluate(1, param), 1e-8);
    EXPECT_NEAR(ddx1, curve.Evaluate(2, param), 1e-8);
    EXPECT_NEAR(x0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(dx0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(ddx0, curve.Evaluate(2, 0.0), 1e-8);
  }
}

TEST(QuarticPolynomialCurve1dTest, IntegratedFromCubicCurve) {
  CubicPolynomialCurve1d cubic_curve(1, 2, 3, 2, 5);
  QuarticPolynomialCurve1d quartic_curve;
  quartic_curve.IntegratedFromCubicCurve(cubic_curve, 0.0);
  for (double value = 0.0; value < 5.1; value += 1) {
    EXPECT_NEAR(quartic_curve.Evaluate(1, value),
                cubic_curve.Evaluate(0, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(2, value),
                cubic_curve.Evaluate(1, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(3, value),
                cubic_curve.Evaluate(2, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(4, value),
                cubic_curve.Evaluate(3, value), 1e-8);
  }
}

TEST(QuarticPolynomialCurve1dTest, DerivedFromQuinticCurve) {
  QuinticPolynomialCurve1d quintic_curve(1, 2, 3, 2, 1, 2, 5);
  QuarticPolynomialCurve1d quartic_curve;
  quartic_curve.DerivedFromQuinticCurve(quintic_curve);
  for (double value = 0.0; value < 5.1; value += 1) {
    EXPECT_NEAR(quartic_curve.Evaluate(0, value),
                quintic_curve.Evaluate(1, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(1, value),
                quintic_curve.Evaluate(2, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(2, value),
                quintic_curve.Evaluate(3, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(3, value),
                quintic_curve.Evaluate(4, value), 1e-8);
    EXPECT_NEAR(quartic_curve.Evaluate(4, value),
                quintic_curve.Evaluate(5, value), 1e-8);
  }
}

TEST(QuarticPolynomialCurve1dTest, FitWithEndPointFirstOrder) {
  QuarticPolynomialCurve1d quartic_curve(1, 2, 4, 2, 1, 3);
  quartic_curve.FitWithEndPointFirstOrder(2, 3, 2, 1, 2, 5);
  EXPECT_NEAR(quartic_curve.Evaluate(0, 0), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(1, 0), 3, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(2, 0), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(0, 5), 1, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(1, 5), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.ParamLength(), 5, 1e-8);
}

TEST(QuarticPolynomialCurve1dTest, FitWithEndPointSecondOrder) {
  QuarticPolynomialCurve1d quartic_curve(1, 2, 4, 2, 1, 3);
  quartic_curve.FitWithEndPointSecondOrder(2, 7, 2, 6, 2, 8);
  EXPECT_NEAR(quartic_curve.Evaluate(0, 0), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(1, 0), 7, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(0, 8), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(1, 8), 6, 1e-8);
  EXPECT_NEAR(quartic_curve.Evaluate(2, 8), 2, 1e-8);
  EXPECT_NEAR(quartic_curve.ParamLength(), 8, 1e-8);
}
}  // namespace planning
}  // namespace apollo
