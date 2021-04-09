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

#include "modules/planning/math/curve1d/cubic_polynomial_curve1d.h"

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(CubicPolynomialCurve1d, Evaluate) {
  {
    double x0 = 0.0;
    double dx0 = 0.0;
    double ddx0 = 0.0;
    double x1 = 10.0;
    double param = 8.0;

    CubicPolynomialCurve1d curve(x0, dx0, ddx0, x1, param);
    EXPECT_NEAR(x1, curve.Evaluate(0, param), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(2, 0.0), 1e-8);
  }
  {
    double x0 = 0.0;
    double dx0 = 0.0;
    double ddx0 = 0.0;
    double x1 = 5.0;
    double param = 3.0;

    CubicPolynomialCurve1d curve(x0, dx0, ddx0, x1, param);
    EXPECT_NEAR(x1, curve.Evaluate(0, param), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(0, curve.Evaluate(2, 0.0), 1e-8);
  }

  {
    double x0 = 1.0;
    double dx0 = 2.0;
    double ddx0 = 3.0;
    double x1 = 5.0;
    double param = 3.0;

    CubicPolynomialCurve1d curve(x0, dx0, ddx0, x1, param);
    EXPECT_NEAR(x1, curve.Evaluate(0, param), 1e-8);
    EXPECT_NEAR(x0, curve.Evaluate(0, 0.0), 1e-8);
    EXPECT_NEAR(dx0, curve.Evaluate(1, 0.0), 1e-8);
    EXPECT_NEAR(ddx0, curve.Evaluate(2, 0.0), 1e-8);
  }
}

}  // namespace planning
}  // namespace apollo
