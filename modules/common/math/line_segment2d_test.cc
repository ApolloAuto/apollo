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

#include "modules/common/math/line_segment2d.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(LineSegment2dTest, Accessors) {
  const LineSegment2d ls({1, 2}, {5, 4});
  EXPECT_NEAR(ls.length(), std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.length_sqr(), 20.0, 1e-5);
  EXPECT_NEAR(ls.center().x(), 3, 1e-5);
  EXPECT_NEAR(ls.center().y(), 3, 1e-5);
  EXPECT_NEAR(ls.heading(), std::atan2(2, 4), 1e-5);
  EXPECT_NEAR(ls.cos_heading(), 4.0 / std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.sin_heading(), 2.0 / std::sqrt(20.0), 1e-5);
  EXPECT_EQ(ls.DebugString(),
            "segment2d ( start = vec2d ( x = 1  y = 2 )  end = vec2d ( x = 5  "
            "y = 4 ) )");
}

TEST(LineSegment2dTest, DistanceTo) {
  const LineSegment2d ls({1, 2}, {5, 4});
  Vec2d nearest_pt;
  EXPECT_NEAR(ls.DistanceTo({0, 0}, &nearest_pt), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(nearest_pt.DistanceTo({0, 0}), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(ls.DistanceTo({0, 0}), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(ls.DistanceSquareTo({0, 0}), 5.0, 1e-5);
  EXPECT_NEAR(ls.DistanceSquareTo({0, 0}, &nearest_pt), 5.0, 1e-5);
  EXPECT_NEAR(ls.DistanceTo({10, 10}, &nearest_pt), std::sqrt(61.0), 1e-5);
  EXPECT_NEAR(nearest_pt.DistanceTo({10, 10}), std::sqrt(61.0), 1e-5);
  EXPECT_NEAR(ls.DistanceTo({1, 2}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(nearest_pt.DistanceTo({1, 2}), 0, 1e-5);
  EXPECT_NEAR(ls.DistanceTo({5, 4}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(nearest_pt.DistanceTo({5, 4}), 0, 1e-5);
  EXPECT_NEAR(ls.DistanceTo({3, 3}, &nearest_pt), 0, 1e-5);
  EXPECT_NEAR(nearest_pt.DistanceTo({3, 3}), 0, 1e-5);
  EXPECT_NEAR(ls.DistanceTo({4, 4}, &nearest_pt), 2.0 / std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(nearest_pt.DistanceTo({4, 4}), 2.0 / std::sqrt(20.0), 1e-5);
}

TEST(LineSegment2dTest, GetPerpendicularFoot) {
  const LineSegment2d ls({1, 2}, {5, 4});
  Vec2d foot_pt;
  EXPECT_NEAR(ls.GetPerpendicularFoot({0, 0}, &foot_pt), 0.6 * std::sqrt(5.0),
              1e-5);
  EXPECT_NEAR(foot_pt.x(), -0.6, 1e-5);
  EXPECT_NEAR(foot_pt.y(), 1.2, 1e-5);
  EXPECT_NEAR(ls.GetPerpendicularFoot({3, 3}, &foot_pt), 0.0, 1e-5);
  EXPECT_NEAR(foot_pt.x(), 3.0, 1e-5);
  EXPECT_NEAR(foot_pt.y(), 3.0, 1e-5);
}

TEST(LineSegment2dTest, ProjectOntoUnit) {
  const LineSegment2d ls({1, 2}, {5, 4});
  EXPECT_NEAR(ls.ProjectOntoUnit({1, 2}), 0.0, 1e-5);
  EXPECT_NEAR(ls.ProjectOntoUnit({5, 4}), std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.ProjectOntoUnit({-2, -3}), -22.0 / std::sqrt(20.0), 1e-5);
  EXPECT_NEAR(ls.ProjectOntoUnit({6, 7}), 30.0 / std::sqrt(20.0), 1e-5);
}

TEST(LineSegment2dTest, GetIntersect) {
  const LineSegment2d ls({1, 2}, {5, 4});
  Vec2d point;
  EXPECT_FALSE(ls.GetIntersect({{1, 3}, {5, 5}}, &point));
  EXPECT_FALSE(ls.GetIntersect({{2, 2}, {6, 4}}, &point));

  EXPECT_TRUE(ls.GetIntersect({{1, 2}, {-3, 0}}, &point));
  EXPECT_NEAR(point.x(), 1, 1e-5);
  EXPECT_NEAR(point.y(), 2, 1e-5);
  EXPECT_TRUE(ls.GetIntersect({{5, 4}, {9, 6}}, &point));
  EXPECT_NEAR(point.x(), 5, 1e-5);
  EXPECT_NEAR(point.y(), 4, 1e-5);

  EXPECT_TRUE(ls.GetIntersect({{3, 0}, {3, 10}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_TRUE(ls.GetIntersect({{3, 10}, {3, 0}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_FALSE(ls.GetIntersect({{3, 5}, {3, 10}}, &point));
  EXPECT_FALSE(ls.GetIntersect({{3, 2}, {3, 0}}, &point));
  EXPECT_TRUE(ls.GetIntersect({{3, 3}, {3, 3}}, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
  EXPECT_FALSE(ls.GetIntersect({{4, 4}, {4, 4}}, &point));
}

TEST(LineSegment2dTest, IsPointIn) {
  const LineSegment2d ls({1, 2}, {5, 4});
  EXPECT_TRUE(ls.IsPointIn({1, 2}));
  EXPECT_TRUE(ls.IsPointIn({5, 4}));
  EXPECT_TRUE(ls.IsPointIn({3, 3}));
  EXPECT_FALSE(ls.IsPointIn({-1, 1}));
  EXPECT_FALSE(ls.IsPointIn({7, 5}));
  EXPECT_FALSE(ls.IsPointIn({0, 0}));
  EXPECT_FALSE(ls.IsPointIn({6, 6}));
}

}  // namespace math
}  // namespace common
}  // namespace apollo
