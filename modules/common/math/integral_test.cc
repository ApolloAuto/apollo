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

#include "modules/common/math/integral.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

namespace {

double LinearFunc(double x) { return 2.0 * x; }

double SquareFunc(double x) { return x * x; }

double CubicFunc(double x) { return x * x * x; }

double SinFunc(double x) { return std::sin(x); }

}  // namespace

TEST(IntegralTest, Integration) {
  double linear_integral = IntegrateByGaussLegendre<5>(LinearFunc, 0.0, 1.0);
  EXPECT_NEAR(linear_integral, 1.0, 1e-5);
  double square_integral = IntegrateByGaussLegendre<5>(SquareFunc, 0.0, 1.0);
  EXPECT_NEAR(square_integral, 1.0 / 3.0, 1e-5);
  double cubic_integral = IntegrateByGaussLegendre<5>(CubicFunc, 0.0, 1.0);
  EXPECT_NEAR(cubic_integral, 1.0 / 4.0, 1e-5);
  double sin_integral = IntegrateByGaussLegendre<5>(SinFunc, 0.0, 0.5 * M_PI);
  EXPECT_NEAR(sin_integral, 1.0, 1e-5);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
