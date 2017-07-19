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

#include "modules/common/math/box2d.h"

#include "gtest/gtest.h"

#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/polygon2d.h"

namespace apollo {
namespace common {
namespace math {

namespace {

bool CheckBoxOverlapSlow(const Box2d &box1, const Box2d &box2,
                         bool *const ambiguous) {
  double headings[4] = {box1.heading(), box1.heading() + M_PI_2, box2.heading(),
                        box2.heading() + M_PI_2};
  *ambiguous = false;
  for (int k = 0; k < 4; ++k) {
    const double heading = headings[k];
    const double cos_heading = cos(heading);
    const double sin_heading = sin(heading);
    std::vector<Vec2d> c1, c2;
    box1.GetAllCorners(&c1);
    box2.GetAllCorners(&c2);
    double s1 = std::numeric_limits<double>::infinity();
    double t1 = -std::numeric_limits<double>::infinity();
    double s2 = std::numeric_limits<double>::infinity();
    double t2 = -std::numeric_limits<double>::infinity();
    for (const auto &p : c1) {
      const double proj = p.x() * cos_heading + p.y() * sin_heading;
      if (proj < s1) s1 = proj;
      if (proj > t1) t1 = proj;
    }
    for (const auto &p : c2) {
      const double proj = p.x() * cos_heading + p.y() * sin_heading;
      if (proj < s2) s2 = proj;
      if (proj > t2) t2 = proj;
    }
    if (std::abs(s1 - t2) <= 1e-5 || std::abs(s2 - t1) <= 1e-5) {
      *ambiguous = true;
    }
    if (s1 > t2 || s2 > t1) {
      return false;
    }
  }
  return true;
}

Box2d box1({0, 0}, 0, 4, 2);
Box2d box2({5, 2}, 0, 4, 2);
Box2d box3(LineSegment2d({2, 3}, {6, 3}), 2);
Box2d box4({7, 8}, M_PI_4, 5.0, 3.0);
Box2d box5({-2, -3}, -M_PI, 0.0, 0.0);
Box2d box6(LineSegment2d({2, 3}, {6, 3}), 0.0);
Box2d box7(AABox2d({4, 5}, 0, 0));

}  // namespace

TEST(Box2dTest, DebugString) {
  Box2d box1({0, 0}, 0, 4, 2);
  Box2d box2({5, 2}, 0, 4, 2);
  EXPECT_EQ(box1.DebugString(),
            "box2d ( center = vec2d ( x = 0  y = 0 )  heading = 0  length = 4  "
            "width = 2 )");
  EXPECT_EQ(box2.DebugString(),
            "box2d ( center = vec2d ( x = 5  y = 2 )  heading = 0  length = 4  "
            "width = 2 )");
}

TEST(Box2dTest, GetAllCorners) {
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

  std::vector<Vec2d> corners2;
  box3.GetAllCorners(&corners2);
  EXPECT_NEAR(corners2[0].x(), 6.0, 1e-5);
  EXPECT_NEAR(corners2[0].y(), 2.0, 1e-5);
  EXPECT_NEAR(corners2[1].x(), 6.0, 1e-5);
  EXPECT_NEAR(corners2[1].y(), 4.0, 1e-5);
  EXPECT_NEAR(corners2[2].x(), 2.0, 1e-5);
  EXPECT_NEAR(corners2[2].y(), 4.0, 1e-5);
  EXPECT_NEAR(corners2[3].x(), 2.0, 1e-5);
  EXPECT_NEAR(corners2[3].y(), 2.0, 1e-5);
}

TEST(Box2dTest, CenterAndLength) {
  EXPECT_NEAR(4, box3.center().x(), 1e-5);
  EXPECT_NEAR(3, box3.center().y(), 1e-5);
  EXPECT_NEAR(4, box3.length(), 1e-5);
  EXPECT_NEAR(2, box3.width(), 1e-5);
  EXPECT_NEAR(2, box3.half_length(), 1e-5);
  EXPECT_NEAR(1, box3.half_width(), 1e-5);
  EXPECT_NEAR(std::hypot(4.0, 2.0), box3.diagonal(), 1e-5);
}

TEST(Box2dTest, HasOverlap) {
  EXPECT_FALSE(box1.HasOverlap(box2));
  EXPECT_FALSE(box1.HasOverlap(box3));
  EXPECT_FALSE(box1.HasOverlap(box4));
  EXPECT_FALSE(box2.HasOverlap(box4));
  EXPECT_FALSE(box3.HasOverlap(box4));

  EXPECT_TRUE(box1.HasOverlap(LineSegment2d({0, 0}, {1, 1})));
  EXPECT_TRUE(box1.HasOverlap(LineSegment2d({0, 0}, {3, 3})));
  EXPECT_TRUE(box1.HasOverlap(LineSegment2d({0, -3}, {0, 3})));
  EXPECT_TRUE(box1.HasOverlap(LineSegment2d({4, 0}, {-4, 0})));
  EXPECT_TRUE(box1.HasOverlap(LineSegment2d({-4, -4}, {4, 4})));
  EXPECT_TRUE(box1.HasOverlap(LineSegment2d({4, -4}, {-4, 4})));
  EXPECT_FALSE(box1.HasOverlap(LineSegment2d({-4, -4}, {4, -4})));
  EXPECT_FALSE(box1.HasOverlap(LineSegment2d({4, -4}, {4, 4})));
}

TEST(Box2dTest, GetAABox) {
  AABox2d aabox1 = box1.GetAABox();
  AABox2d aabox2 = box2.GetAABox();
  AABox2d aabox3 = box3.GetAABox();
  AABox2d aabox4 = box4.GetAABox();
  EXPECT_NEAR(aabox1.center_x(), 0.0, 1e-5);
  EXPECT_NEAR(aabox1.center_y(), 0.0, 1e-5);
  EXPECT_NEAR(aabox1.length(), 4.0, 1e-5);
  EXPECT_NEAR(aabox1.width(), 2.0, 1e-5);
  EXPECT_NEAR(aabox2.center_x(), 5.0, 1e-5);
  EXPECT_NEAR(aabox2.center_y(), 2.0, 1e-5);
  EXPECT_NEAR(aabox2.length(), 4.0, 1e-5);
  EXPECT_NEAR(aabox2.width(), 2.0, 1e-5);
  EXPECT_NEAR(aabox3.center_x(), 4.0, 1e-5);
  EXPECT_NEAR(aabox3.center_y(), 3.0, 1e-5);
  EXPECT_NEAR(aabox3.length(), 4.0, 1e-5);
  EXPECT_NEAR(aabox3.width(), 2.0, 1e-5);
  EXPECT_NEAR(aabox4.center_x(), 7.0, 1e-5);
  EXPECT_NEAR(aabox4.center_y(), 8.0, 1e-5);
  EXPECT_NEAR(aabox4.length(), 4.0 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(aabox4.width(), 4.0 * sqrt(2.0), 1e-5);
}

TEST(Box2dTest, DistanceTo) {
  EXPECT_NEAR(box1.DistanceTo({3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({-3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, 2}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, -2}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({0, -1}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo({-1, 0}), 0.0, 1e-5);

  EXPECT_NEAR(box1.DistanceTo(LineSegment2d({-4, -4}, {4, 4})), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo(LineSegment2d({4, -4}, {-4, 4})), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo(LineSegment2d({0, 2}, {4, 4})), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceTo(LineSegment2d({2, 2}, {3, 1})),
              std::sqrt(2.0) / 2.0, 1e-5);
}

TEST(Box2dTest, IsPointIn) {
  EXPECT_TRUE(box1.IsPointIn({0, 0}));
  EXPECT_TRUE(box1.IsPointIn({1, 0.5}));
  EXPECT_TRUE(box1.IsPointIn({-0.5, -1}));
  EXPECT_TRUE(box1.IsPointIn({2, 1}));
  EXPECT_FALSE(box1.IsPointIn({-3, 0}));
  EXPECT_FALSE(box1.IsPointIn({0, 2}));
  EXPECT_FALSE(box1.IsPointIn({-4, -2}));
}

TEST(Box2dTest, IsPointOnBoundary) {
  EXPECT_FALSE(box1.IsPointOnBoundary({0, 0}));
  EXPECT_FALSE(box1.IsPointOnBoundary({1, 0.5}));
  EXPECT_TRUE(box1.IsPointOnBoundary({-0.5, -1}));
  EXPECT_TRUE(box1.IsPointOnBoundary({2, 0.5}));
  EXPECT_TRUE(box1.IsPointOnBoundary({-2, 1}));
  EXPECT_FALSE(box1.IsPointOnBoundary({-3, 0}));
  EXPECT_FALSE(box1.IsPointOnBoundary({0, 2}));
  EXPECT_FALSE(box1.IsPointOnBoundary({-4, -2}));
}

TEST(Box2dTest, RotateFromCenterAndShift) {
  Box2d box({0, 0}, 0, 4, 2);
  std::vector<Vec2d> corners;
  EXPECT_NEAR(box.heading(), 0.0, 1e-5);
  box.RotateFromCenter(M_PI_2);
  EXPECT_NEAR(box.heading(), M_PI_2, 1e-5);
  box.GetAllCorners(&corners);
  EXPECT_NEAR(corners[0].x(), 1.0, 1e-5);
  EXPECT_NEAR(corners[0].y(), 2.0, 1e-5);
  EXPECT_NEAR(corners[1].x(), -1.0, 1e-5);
  EXPECT_NEAR(corners[1].y(), 2.0, 1e-5);
  EXPECT_NEAR(corners[2].x(), -1.0, 1e-5);
  EXPECT_NEAR(corners[2].y(), -2.0, 1e-5);
  EXPECT_NEAR(corners[3].x(), 1.0, 1e-5);
  EXPECT_NEAR(corners[3].y(), -2.0, 1e-5);

  box.Shift({30, 40});
  box.GetAllCorners(&corners);
  EXPECT_NEAR(corners[0].x(), 31.0, 1e-5);
  EXPECT_NEAR(corners[0].y(), 42.0, 1e-5);
  EXPECT_NEAR(corners[1].x(), 29.0, 1e-5);
  EXPECT_NEAR(corners[1].y(), 42.0, 1e-5);
  EXPECT_NEAR(corners[2].x(), 29.0, 1e-5);
  EXPECT_NEAR(corners[2].y(), 38.0, 1e-5);
  EXPECT_NEAR(corners[3].x(), 31.0, 1e-5);
  EXPECT_NEAR(corners[3].y(), 38.0, 1e-5);
}

TEST(Box2dTest, TestByRandom) {
  bool ambiguous = false;
  for (int iter = 0; iter < 10000; ++iter) {
    const double x1 = RandomDouble(-10, 10);
    const double y1 = RandomDouble(-10, 10);
    const double x2 = RandomDouble(-10, 10);
    const double y2 = RandomDouble(-10, 10);
    const double heading1 = RandomDouble(0, M_PI * 2.0);
    const double heading2 = RandomDouble(0, M_PI * 2.0);
    const double l1 = RandomDouble(1, 5);
    const double l2 = RandomDouble(1, 5);
    const double w1 = RandomDouble(1, 5);
    const double w2 = RandomDouble(1, 5);
    const Box2d box1({x1, y1}, heading1, l1, w1);
    const Box2d box2({x2, y2}, heading2, l2, w2);
    bool overlap = CheckBoxOverlapSlow(box1, box2, &ambiguous);
    if (!ambiguous) {
      EXPECT_EQ(overlap, box1.HasOverlap(box2));
    }
  }
  for (int iter = 0; iter < 100; ++iter) {
    const double x = RandomDouble(-10, 10);
    const double y = RandomDouble(-10, 10);
    const double heading = RandomDouble(0, M_PI * 2.0);
    const double length = RandomDouble(1, 5);
    const double width = RandomDouble(1, 5);
    const Box2d box({x, y}, heading, length, width);
    const Polygon2d poly(box);
    for (int iter2 = 0; iter2 < 1000; ++iter2) {
      const double x = RandomDouble(-20, 20);
      const double y = RandomDouble(-20, 20);
      EXPECT_EQ(box.IsPointIn({x, y}), poly.IsPointIn({x, y}));
      EXPECT_EQ(box.IsPointOnBoundary({x, y}), poly.IsPointOnBoundary({x, y}));
      EXPECT_NEAR(box.DistanceTo({x, y}), poly.DistanceTo({x, y}), 1e-3);

      const double other_x = RandomDouble(-20, 20);
      const double other_y = RandomDouble(-20, 20);
      const LineSegment2d segment({x, y}, {other_x, other_y});
      EXPECT_EQ(box.HasOverlap(segment), poly.HasOverlap(segment));
      EXPECT_NEAR(box.DistanceTo(segment), poly.DistanceTo(segment), 1e-3);
    }
  }
}

}  // namespace math
}  // namespace common
}  // namespace apollo
