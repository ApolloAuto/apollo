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
#include "modules/common/math/linear_interpolation.h"

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(LinearInterpolationTest, LerpOneDim) {
  double t0 = 0.0;
  double t1 = 1.0;
  double x0 = 2.0;
  double x1 = 4.0;

  EXPECT_NEAR(lerp(x0, t0, x1, t1, 0.4), 2.8, 1e-6);
  EXPECT_NEAR(lerp(x0, t0, x1, t1, 0.9), 3.8, 1e-6);
  EXPECT_NEAR(lerp(x0, t0, x1, t1, 1.5), 5.0, 1e-6);
  EXPECT_NEAR(lerp(x0, t0, x1, t1, -0.3), 1.4, 1e-6);
}

TEST(LinearInterpolationTest, LerpTwoDim) {
  double t0 = 0.0;
  double t1 = 1.0;

  Eigen::Vector2d x0(2.0, 1.0);
  Eigen::Vector2d x1(4.0, 5.0);

  Eigen::Vector2d x = lerp(x0, t0, x1, t1, 0.4);
  EXPECT_NEAR(x.x(), 2.8, 1e-6);
  EXPECT_NEAR(x.y(), 2.6, 1e-6);

  x = lerp(x0, t0, x1, t1, 1.2);
  EXPECT_NEAR(x.x(), 4.4, 1e-6);
  EXPECT_NEAR(x.y(), 5.8, 1e-6);

  x = lerp(x0, t0, x1, t1, -0.5);
  EXPECT_NEAR(x.x(), 1.0, 1e-6);
  EXPECT_NEAR(x.y(), -1.0, 1e-6);
}

TEST(LinearInterpolationTest, SlerpCaseOne) {
  double t0 = 0.0;
  double t1 = 1.0;
  double a0 = -2.0;
  double a1 = 8.5;

  EXPECT_NEAR(slerp(a0, t0, a1, t1, 0.4), -2.827, 1e-3);
}

TEST(LinearInterpolationTest, SlerpCaseTwo) {
  double t0 = 0.0;
  double t1 = 1.0;
  double a0 = 3.00;
  double a1 = -3.00;

  EXPECT_NEAR(slerp(a0, t0, a1, t1, 0.5001), -3.1416, 1e-3);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
