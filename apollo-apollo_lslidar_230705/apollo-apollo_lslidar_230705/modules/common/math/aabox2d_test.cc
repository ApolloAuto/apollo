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

#include "modules/common/math/aabox2d.h"

#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(AABox2dTest, GetAllCorners) {
  AABox2d box1({0, 0}, 4, 2);
  std::vector<Vec2d> corners1;
  box1.GetAllCorners(&corners1);
  EXPECT_NEAR(corners1[0].x(), 2.0, 1e-5);
  EXPECT_NEAR(corners1[0].y(), -1.0, 1e-5);
  EXPECT_NEAR(corners1[1].x(), 2.0, 1e-5);
  EXPECT_NEAR(corners1[1].y(), 1.0, 1e-5);
  EXPECT_NEAR(corners1[2].x(), -2.0, 1e-5);
  EXPECT_NEAR(corners1[2].y(), 1.0, 1e-5);
  EXPECT_NEAR(corners1[3].x(), -2.0, 1e-5);
  EXPECT_NEAR(corners1[3].y(), -1.0, 1e-5);
  EXPECT_EQ(
      box1.DebugString(),
      "aabox2d ( center = vec2d ( x = 0  y = 0 )  length = 4  width = 2 )");
  std::vector<Vec2d> corners2;

  AABox2d box2({3, 1}, {7, 3});
  box2.GetAllCorners(&corners2);
  EXPECT_NEAR(corners2[0].x(), 7.0, 1e-5);
  EXPECT_NEAR(corners2[0].y(), 1.0, 1e-5);
  EXPECT_NEAR(corners2[1].x(), 7.0, 1e-5);
  EXPECT_NEAR(corners2[1].y(), 3.0, 1e-5);
  EXPECT_NEAR(corners2[2].x(), 3.0, 1e-5);
  EXPECT_NEAR(corners2[2].y(), 3.0, 1e-5);
  EXPECT_NEAR(corners2[3].x(), 3.0, 1e-5);
  EXPECT_NEAR(corners2[3].y(), 1.0, 1e-5);
  EXPECT_EQ(
      box2.DebugString(),
      "aabox2d ( center = vec2d ( x = 5  y = 2 )  length = 4  width = 2 )");
}

TEST(AABox2dTest, CenterAndLengths) {
  AABox2d box1({0, 0}, 10, 10);
  EXPECT_NEAR(box1.center_x(), 0.0, 1e-5);
  EXPECT_NEAR(box1.center_y(), 0.0, 1e-5);
  EXPECT_NEAR(box1.length(), 10.0, 1e-5);
  EXPECT_NEAR(box1.width(), 10.0, 1e-5);
  EXPECT_NEAR(box1.half_length(), 5.0, 1e-5);
  EXPECT_NEAR(box1.half_width(), 5.0, 1e-5);

  AABox2d box2({{0, 2}, {0, -6}, {3, 0}, {1, 0}});
  EXPECT_NEAR(box2.center_x(), 1.5, 1e-5);
  EXPECT_NEAR(box2.center_y(), -2.0, 1e-5);
  EXPECT_NEAR(box2.length(), 3.0, 1e-5);
  EXPECT_NEAR(box2.width(), 8.0, 1e-5);
  EXPECT_NEAR(box2.half_length(), 1.5, 1e-5);
  EXPECT_NEAR(box2.half_width(), 4.0, 1e-5);
}

TEST(AABox2dTest, HasOverlap) {
  AABox2d box1({0, 0}, 4, 2);
  AABox2d box2({3, 1}, {7, 3});
  AABox2d box3({0, 0}, 10, 10);
  EXPECT_FALSE(box1.HasOverlap(box2));
  EXPECT_TRUE(box1.HasOverlap(box3));
  EXPECT_TRUE(box2.HasOverlap(box3));
}

TEST(AABox2dTest, DistanceTo) {
  AABox2d box({0, 0}, 4, 2);
  EXPECT_NEAR(box.DistanceTo({3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box.DistanceTo({-3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box.DistanceTo({0, 2}), 1.0, 1e-5);
  EXPECT_NEAR(box.DistanceTo({0, -2}), 1.0, 1e-5);
  EXPECT_NEAR(box.DistanceTo({0, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box.DistanceTo({0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(box.DistanceTo({1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box.DistanceTo({0, -1}), 0.0, 1e-5);
  EXPECT_NEAR(box.DistanceTo({-1, 0}), 0.0, 1e-5);
}

TEST(AABox2dTest, IsPointIn) {
  AABox2d box({0, 0}, 4, 2);
  EXPECT_TRUE(box.IsPointIn({0, 0}));
  EXPECT_TRUE(box.IsPointIn({1, 0.5}));
  EXPECT_TRUE(box.IsPointIn({-0.5, -1}));
  EXPECT_TRUE(box.IsPointIn({2, 1}));
  EXPECT_FALSE(box.IsPointIn({-3, 0}));
  EXPECT_FALSE(box.IsPointIn({0, 2}));
  EXPECT_FALSE(box.IsPointIn({-4, -2}));
}

TEST(AABox2dTest, IsPointOnBoundary) {
  AABox2d box({0, 0}, 4, 2);
  EXPECT_FALSE(box.IsPointOnBoundary({0, 0}));
  EXPECT_FALSE(box.IsPointOnBoundary({1, 0.5}));
  EXPECT_TRUE(box.IsPointOnBoundary({-0.5, -1}));
  EXPECT_TRUE(box.IsPointOnBoundary({2, 0.5}));
  EXPECT_TRUE(box.IsPointOnBoundary({-2, 1}));
  EXPECT_FALSE(box.IsPointOnBoundary({-3, 0}));
  EXPECT_FALSE(box.IsPointOnBoundary({0, 2}));
  EXPECT_FALSE(box.IsPointOnBoundary({-4, -2}));
}

TEST(AABox2dTest, MinMax) {
  AABox2d box1({0, 0}, 4, 2);
  EXPECT_NEAR(box1.min_x(), -2, 1e-5);
  EXPECT_NEAR(box1.max_x(), 2, 1e-5);
  EXPECT_NEAR(box1.min_y(), -1, 1e-5);
  EXPECT_NEAR(box1.max_y(), 1, 1e-5);

  AABox2d box2({3, 1}, {7, 3});
  EXPECT_NEAR(box2.min_x(), 3, 1e-5);
  EXPECT_NEAR(box2.max_x(), 7, 1e-5);
  EXPECT_NEAR(box2.min_y(), 1, 1e-5);
  EXPECT_NEAR(box2.max_y(), 3, 1e-5);
}

TEST(AABox2dTest, Shift) {
  AABox2d box({0, 0}, 4, 2);
  box.Shift({30, 40});
  std::vector<Vec2d> corners;
  box.GetAllCorners(&corners);
  EXPECT_NEAR(corners[0].x(), 32.0, 1e-5);
  EXPECT_NEAR(corners[0].y(), 39.0, 1e-5);
  EXPECT_NEAR(corners[1].x(), 32.0, 1e-5);
  EXPECT_NEAR(corners[1].y(), 41.0, 1e-5);
  EXPECT_NEAR(corners[2].x(), 28.0, 1e-5);
  EXPECT_NEAR(corners[2].y(), 41.0, 1e-5);
  EXPECT_NEAR(corners[3].x(), 28.0, 1e-5);
  EXPECT_NEAR(corners[3].y(), 39.0, 1e-5);
}

TEST(AABox2dTest, MergeFrom) {
  AABox2d box({3, 1}, {7, 3});
  box.MergeFrom(AABox2d({5, -1}, {10, 7}));
  EXPECT_NEAR(box.center_x(), 6.5, 1e-5);
  EXPECT_NEAR(box.center_y(), 3, 1e-5);
  EXPECT_NEAR(box.length(), 7, 1e-5);
  EXPECT_NEAR(box.width(), 8, 1e-5);
  EXPECT_NEAR(box.half_length(), 3.5, 1e-5);
  EXPECT_NEAR(box.half_width(), 4, 1e-5);

  box.MergeFrom({6, 6});
  EXPECT_NEAR(box.center_x(), 6.5, 1e-5);
  EXPECT_NEAR(box.center_y(), 3, 1e-5);
  EXPECT_NEAR(box.length(), 7, 1e-5);
  EXPECT_NEAR(box.width(), 8, 1e-5);
  EXPECT_NEAR(box.half_length(), 3.5, 1e-5);
  EXPECT_NEAR(box.half_width(), 4, 1e-5);

  box.MergeFrom({-5, 20});
  EXPECT_NEAR(box.center_x(), 2.5, 1e-5);
  EXPECT_NEAR(box.center_y(), 9.5, 1e-5);
  EXPECT_NEAR(box.length(), 15, 1e-5);
  EXPECT_NEAR(box.width(), 21, 1e-5);
  EXPECT_NEAR(box.half_length(), 7.5, 1e-5);
  EXPECT_NEAR(box.half_width(), 10.5, 1e-5);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
