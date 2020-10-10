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

#include "modules/common/math/search.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

namespace {

double LinearFunc(double x) { return 2.0 * x; }

double SquareFunc(double x) { return x * x; }

double CubicFunc(double x) { return (x - 1.0) * (x - 2.0) * (x - 3.0); }

double SinFunc(double x) { return std::sin(x); }

}  // namespace

TEST(SearchTest, GoldenSectionSearch) {
  double linear_argmin = GoldenSectionSearch(LinearFunc, 0.0, 1.0, 1e-6);
  EXPECT_NEAR(linear_argmin, 0.0, 1e-5);
  double square_argmin = GoldenSectionSearch(SquareFunc, -1.0, 2.0, 1e-6);
  EXPECT_NEAR(square_argmin, 0.0, 1e-5);
  double cubic_argmin_1 = GoldenSectionSearch(CubicFunc, 0.0, 1.5, 1e-6);
  EXPECT_NEAR(cubic_argmin_1, 0.0, 1e-5);
  double cubic_argmin_2 = GoldenSectionSearch(CubicFunc, 1.0, 1.8, 1e-6);
  EXPECT_NEAR(cubic_argmin_2, 1.0, 1e-5);
  double cubic_argmin_3 = GoldenSectionSearch(CubicFunc, 2.0, 3.0, 1e-6);
  EXPECT_NEAR(cubic_argmin_3, 2.0 + 1.0 / std::sqrt(3.0), 1e-5);
  double sin_argmin = GoldenSectionSearch(SinFunc, 0.0, 2 * M_PI, 1e-6);
  EXPECT_NEAR(sin_argmin, 1.5 * M_PI, 1e-5);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
