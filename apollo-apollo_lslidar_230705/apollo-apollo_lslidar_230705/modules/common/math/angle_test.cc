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

#include "modules/common/math/angle.h"

#include <cmath>

#include "gtest/gtest.h"
#include "modules/common/math/sin_table.h"

namespace apollo {
namespace common {
namespace math {

TEST(Angle, SIN_TABLE) {
  EXPECT_FLOAT_EQ(0.0f, SIN_TABLE[0]);
  EXPECT_FLOAT_EQ(1.0f, SIN_TABLE[16384]);
}

TEST(Angle, Angle8) {
  auto a = Angle8::from_deg(90.0);
  EXPECT_DOUBLE_EQ(90.0, a.to_deg());
  EXPECT_DOUBLE_EQ(M_PI_2, a.to_rad());
  EXPECT_FLOAT_EQ(1.0f, sin(a));
  EXPECT_FLOAT_EQ(0.0f, cos(a));
}

TEST(Angle, Angle16) {
  auto a = Angle16(1);
  EXPECT_DOUBLE_EQ(180.0 / 32768, a.to_deg());

  a = Angle16::from_deg(-150.0 - 360.0);
  EXPECT_NEAR(-0.5f, sin(a), 1e-4);
  EXPECT_NEAR(-0.5 * sqrt(3), cos(a), 1e-4);
}

TEST(Angle, Angle32) {
  auto a = Angle32::from_rad(1.0);
  EXPECT_NEAR(180 / M_PI, a.to_deg(), 1e-7);
  EXPECT_NEAR(1.0, a.to_rad(), 1e-9);
}

TEST(Angle, operators) {
  auto a = Angle16::from_deg(100.0);
  auto b = a;
  a += b;
  a *= 0.5;
  a = 7 * (a + b * 0.7);
  a /= 1.1;
  EXPECT_DOUBLE_EQ(-63.65478515625, a.to_deg());
}

}  // namespace math
}  // namespace common
}  // namespace apollo
