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

#include "modules/common/math/vec2d_utils.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(Vec2DUtilsTest, NomralCases) {
  Vec2D pt0 = Vec2DCtor(2, 3);
  EXPECT_NEAR(VecLength(pt0), std::sqrt(13.0), 1e-5);
  EXPECT_NEAR(VecLengthSquare(pt0), 13.0, 1e-5);
  EXPECT_NEAR(VecAngle(pt0), std::atan2(3, 2), 1e-5);

  Vec2D pt1 = Vec2DCtor(0, 0);  // pt1 = (0, 0)
  EXPECT_NEAR(VecDistance(pt0, pt1), std::sqrt(13.0), 1e-5);
  EXPECT_NEAR(VecDistanceSquare(pt0, pt1), 13.0, 1e-5);

  pt1 = Vec2DCtor(0, 2);
  EXPECT_NEAR(VecDistance(pt0, pt1), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(VecDistanceSquare(pt0, pt1), 5.0, 1e-5);

  pt1 = Vec2DCtor(4, 5);
  EXPECT_NEAR(VecCrossProd(pt0, pt1), -2, 1e-5);
  EXPECT_NEAR(VecInnerProd(pt0, pt1), 23, 1e-5);
  EXPECT_NEAR(VecLength(pt1), std::sqrt(41.0), 1e-5);
  EXPECT_NEAR(VecLengthSquare(pt1), 41.0, 1e-5);

  VecNormalize(&pt1);  // |pt1| = 1
  EXPECT_NEAR(VecLength(pt1), 1.0, 1e-5);
  EXPECT_NEAR(pt1.x(), 4.0 / std::sqrt(41.0), 1e-5);
  EXPECT_NEAR(pt1.y(), 5.0 / std::sqrt(41.0), 1e-5);

  pt0 = Vec2DCtor(0.5, 1.5);
  pt1 = Vec2DCtor(2.5, 3.5);

  const Vec2D d = pt0 + pt1;
  EXPECT_NEAR(d.x(), 3.0, 1e-5);
  EXPECT_NEAR(d.y(), 5.0, 1e-5);

  const Vec2D e = pt0 - pt1;
  EXPECT_NEAR(e.x(), -2.0, 1e-5);
  EXPECT_NEAR(e.y(), -2.0, 1e-5);

  const Vec2D f = d / 2.0;
  EXPECT_NEAR(f.x(), 1.5, 1e-5);
  EXPECT_NEAR(f.y(), 2.5, 1e-5);

  const Vec2D g = e * (-3.0);
  EXPECT_NEAR(g.x(), 6.0, 1e-5);
  EXPECT_NEAR(g.y(), 6.0, 1e-5);

  const Vec2D unit_pt = Vec2DUnit(M_PI_4);
  EXPECT_NEAR(unit_pt.x(), std::sqrt(2.0) / 2.0, 1e-5);
  EXPECT_NEAR(unit_pt.y(), std::sqrt(2.0) / 2.0, 1e-5);
  EXPECT_NEAR(VecAngle(unit_pt), M_PI_4, 1e-5);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
