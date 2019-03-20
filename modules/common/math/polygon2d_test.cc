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

#include "modules/common/math/polygon2d.h"

#include <algorithm>
#include <string>

#include "gtest/gtest.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/math_utils.h"

namespace apollo {
namespace common {
namespace math {

namespace {

bool ProjectByXSlow(const std::vector<Vec2d> &points, double x,
                    double *const min_y, double *const max_y) {
  *min_y = std::numeric_limits<double>::infinity();
  *max_y = -std::numeric_limits<double>::infinity();
  for (const Vec2d &p1 : points) {
    if (p1.x() < x) {
      for (const Vec2d &p2 : points) {
        if (p2.x() > x) {
          const double y = ((p2.x() - x) * p1.y() + (x - p1.x()) * p2.y()) /
                           (p2.x() - p1.x());
          *min_y = std::min(*min_y, y);
          *max_y = std::max(*max_y, y);
        }
      }
    }
  }
  return *min_y <= *max_y;
}

}  // namespace

TEST(Polygon2dTest, polygon_IsPointIn) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {1, 1}));
  EXPECT_EQ(poly1.DebugString(),
            "polygon2d (  num_points = 4  points = (vec2d ( x = 1  y = 0 ) "
            "vec2d ( x = 1  y = 1 ) vec2d ( x = 0  y = 1 ) vec2d ( x = 0  y = "
            "0 ) )  convex  area = 1 )");
  EXPECT_TRUE(poly1.IsPointIn({0.5, 0.5}));
  EXPECT_TRUE(poly1.IsPointIn({0.2, 0.2}));
  EXPECT_TRUE(poly1.IsPointIn({0.2, 0.8}));
  EXPECT_TRUE(poly1.IsPointIn({0.8, 0.2}));
  EXPECT_TRUE(poly1.IsPointIn({0.8, 0.8}));

  EXPECT_TRUE(poly1.IsPointOnBoundary({0.0, 0.0}));
  EXPECT_TRUE(poly1.IsPointIn({0.0, 0.0}));
  EXPECT_TRUE(poly1.IsPointOnBoundary({0.0, 0.5}));
  EXPECT_TRUE(poly1.IsPointIn({0.0, 0.5}));
  EXPECT_TRUE(poly1.IsPointOnBoundary({1.0, 0.5}));
  EXPECT_TRUE(poly1.IsPointIn({1.0, 0.5}));

  EXPECT_FALSE(poly1.IsPointIn({-0.2, 0.5}));
  EXPECT_FALSE(poly1.IsPointIn({1.2, 0.5}));
  EXPECT_FALSE(poly1.IsPointIn({0.5, -0.2}));
  EXPECT_FALSE(poly1.IsPointIn({0.5, 1.2}));

  EXPECT_FALSE(poly1.IsPointIn({0, -0.1}));
  EXPECT_FALSE(poly1.IsPointIn({-0.1, 0}));
  EXPECT_FALSE(poly1.IsPointIn({1.0, -0.1}));
  EXPECT_FALSE(poly1.IsPointIn({-0.1, 1.0}));
  EXPECT_FALSE(poly1.IsPointIn({0, 1.1}));
  EXPECT_FALSE(poly1.IsPointIn({1.1, 0}));
  EXPECT_FALSE(poly1.IsPointIn({1.0, 1.1}));
  EXPECT_FALSE(poly1.IsPointIn({1.1, 1.0}));

  const Polygon2d poly2({{0, 1}, {1, 0}, {0, -1}, {-1, 0}});
  EXPECT_TRUE(poly2.IsPointIn({0, 0}));
  EXPECT_TRUE(poly2.IsPointIn({0, 0.9}));
  EXPECT_TRUE(poly2.IsPointIn({0.9, 0}));
  EXPECT_TRUE(poly2.IsPointIn({0, -0.9}));
  EXPECT_TRUE(poly2.IsPointIn({-0.9, 0}));

  EXPECT_FALSE(poly2.IsPointIn({0, 1.1}));
  EXPECT_FALSE(poly2.IsPointIn({1.1, 0}));
  EXPECT_FALSE(poly2.IsPointIn({0, -1.1}));
  EXPECT_FALSE(poly2.IsPointIn({-1.1, 0}));

  const Polygon2d poly3({{4, 4}, {5, 6}, {6, 6}});
  EXPECT_FALSE(poly3.IsPointIn({5, 4.5}));
  EXPECT_FALSE(poly3.IsPointOnBoundary({5, 4.5}));
  EXPECT_TRUE(poly3.IsPointIn({5, 5}));
  EXPECT_TRUE(poly3.IsPointOnBoundary({5, 5}));
  EXPECT_TRUE(poly3.IsPointIn({5, 5.5}));
  EXPECT_FALSE(poly3.IsPointOnBoundary({5, 5.5}));
  EXPECT_TRUE(poly3.IsPointIn({5, 6}));
  EXPECT_TRUE(poly3.IsPointOnBoundary({5, 6}));
  EXPECT_FALSE(poly3.IsPointIn({5, 6.5}));
  EXPECT_FALSE(poly3.IsPointOnBoundary({5, 6.5}));

  // Concave polygons.
  const Polygon2d poly4({{0, 0}, {2, 0}, {2, 2}, {1, 1}, {0, 2}});
  EXPECT_TRUE(poly4.IsPointIn({0.5, 1.5}));
  EXPECT_TRUE(poly4.IsPointOnBoundary({0.5, 1.5}));
  EXPECT_FALSE(poly4.IsPointIn({1.0, 1.5}));
  EXPECT_TRUE(poly4.IsPointIn({1.5, 1.5}));
  EXPECT_TRUE(poly4.IsPointOnBoundary({1.5, 1.5}));

  const Polygon2d poly5(
      {{0, 0}, {4, 0}, {4, 2}, {3, 2}, {2, 1}, {1, 2}, {0, 2}});
  EXPECT_FALSE(poly5.IsPointIn({-0.5, 2.0}));
  EXPECT_TRUE(poly5.IsPointIn({0.5, 2.0}));
  EXPECT_FALSE(poly5.IsPointIn({1.5, 2.0}));
  EXPECT_FALSE(poly5.IsPointIn({2.0, 2.0}));
  EXPECT_FALSE(poly5.IsPointIn({2.5, 2.0}));
  EXPECT_TRUE(poly5.IsPointIn({3.5, 2.0}));
  EXPECT_FALSE(poly5.IsPointIn({4.5, 2.0}));
}

TEST(Polygon2dTest, DistanceToPoint) {
  const Box2d box1(Box2d::CreateAABox({0, 0}, {1, 1}));
  const Polygon2d poly1(box1);
  EXPECT_NEAR(poly1.DistanceTo({0.5, 0.5}), 0.0, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({-0.2, 0.5}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.2, 0.5}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({0.5, -0.2}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({0.5, 1.2}), 0.2, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({0, -0.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({-0.1, 0}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.0, -0.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({-0.1, 1.0}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({0, 1.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.1, 0}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.0, 1.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.1, 1.0}), 0.1, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({-0.1, -0.1}), 0.1 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({-0.1, 1.1}), 0.1 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.1, -0.1}), 0.1 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({1.1, 1.1}), 0.1 * sqrt(2.0), 1e-5);

  for (int iter = 0; iter < 10000; ++iter) {
    const double x = RandomDouble(-10, 10);
    const double y = RandomDouble(-10, 10);
    EXPECT_NEAR(poly1.DistanceTo({x, y}), box1.DistanceTo({x, y}), 1e-5);
  }
  for (int iter = 0; iter < 100; ++iter) {
    const double center_x = RandomDouble(-10, 10);
    const double center_y = RandomDouble(-10, 10);
    const double heading = RandomDouble(0, M_PI * 2.0);
    const double length = RandomDouble(1, 5);
    const double width = RandomDouble(1, 5);
    const Box2d box({center_x, center_y}, heading, length, width);
    const Polygon2d polygon(box);
    for (int iter2 = 0; iter2 < 100; ++iter2) {
      const double x = RandomDouble(-20, 20);
      const double y = RandomDouble(-20, 20);
      EXPECT_NEAR(polygon.DistanceTo({x, y}), box.DistanceTo({x, y}), 1e-5);
    }
  }

  const Polygon2d poly2({{0, 1}, {1, 0}, {0, -1}, {-1, 0}});
  EXPECT_NEAR(poly2.DistanceTo({0, 0}), 0.0, 1e-5);

  EXPECT_NEAR(poly2.DistanceTo({0, 1.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({1.1, 0}), 0.1, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({0, -1.1}), 0.1, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({-1.1, 0}), 0.1, 1e-5);

  EXPECT_NEAR(poly2.DistanceTo({0.5, 0.5}), 0.0, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({0.6, 0.6}), 0.1 * sqrt(2.0), 1e-5);

  const Polygon2d poly3(
      {{0, 0}, {4, 0}, {4, 2}, {3, 2}, {2, 1}, {1, 2}, {0, 2}});
  EXPECT_NEAR(poly3.DistanceTo({-0.5, 2.0}), 0.5, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({0.5, 2.0}), 0.0, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({1.5, 2.0}), 0.5 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({2.0, 2.0}), 1.0 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({2.5, 2.0}), 0.5 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({3.5, 2.0}), 0.0, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({4.5, 2.0}), 0.5, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({-0.5, 1.0}), 0.5, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({2.0, 1.0}), 0.0, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({4.5, 1.0}), 0.5, 1e-5);
}

TEST(Polygon2dTest, DistanceToLineSegment) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {1, 1}));
  EXPECT_NEAR(poly1.DistanceTo({{0.5, 0.5}, {1.0, 1.0}}), 0.0, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{-0.2, 0.5}, {1.2, 0.5}}), 0.0, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{-2.0, -2.0}, {2.0, 2.0}}), 0.0, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{-0.2, 0.5}, {-0.2, 0.8}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{1.2, 0.5}, {1.2, 0.3}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, -0.2}, {0.8, -0.2}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, 1.2}, {0.3, 1.2}}), 0.2, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{-0.3, 0.5}, {-0.2, 0.8}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{1.2, 0.5}, {1.3, 0.3}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, -0.3}, {0.8, -0.2}}), 0.2, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, 1.2}, {0.3, 1.3}}), 0.2, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{0, -0.1}, {-0.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{1.0, 1.1}, {1.1, 1.0}}), 0.1 / sqrt(2.0),
              1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{-10.0, 0.5}, {2.0, 0.5}}), 0.0, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{-1.0, 0.5}, {10.0, 0.5}}), 0.0, 1e-5);

  EXPECT_NEAR(poly1.DistanceTo({{-1.0, 2.0}, {-1.0, 2.0}}), sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly1.DistanceTo({{0.5, 0.5}, {0.5, 0.5}}), 0.0, 1e-5);

  const Polygon2d poly2({{0, 1}, {1, 0}, {0, -1}, {-1, 0}});
  EXPECT_NEAR(poly2.DistanceTo({{-2, 0}, {2, 0}}), 0.0, 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0, -2}, {0, 2}}), 0.0, 1e-5);

  EXPECT_NEAR(poly2.DistanceTo({{0, 1.1}, {1.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0, 1.1}, {-1.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0, -1.1}, {1.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0, -1.1}, {-1.1, 0}}), 0.1 / sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{0.6, 0.6}, {0.7, 0.7}}), 0.1 * sqrt(2.0),
              1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{-0.6, -0.6}, {-0.7, -0.7}}), 0.1 * sqrt(2.0),
              1e-5);
  EXPECT_NEAR(poly2.DistanceTo({{-0.6, -0.6}, {0.7, 0.7}}), 0.0, 1e-5);

  const Polygon2d poly3({{0, 0}, {2, 0}, {2, 2}, {1, 1}, {0, 2}});
  EXPECT_NEAR(poly3.DistanceTo({{-2, 0}, {2, 0}}), 0.0, 1e-5);
  EXPECT_NEAR(poly3.DistanceTo({{0.7, 2.0}, {1.2, 2.0}}), 0.7 / sqrt(2.0),
              1e-5);
  EXPECT_NEAR(poly3.DistanceTo({{0.7, 2.0}, {1.4, 2.0}}), 0.6 / sqrt(2.0),
              1e-5);
  EXPECT_NEAR(poly3.DistanceTo({{0.7, 1.5}, {1.6, 1.5}}), 0.0, 1e-5);
}

TEST(Polygon2dTest, DistanceToPolygon) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {1, 1}));
  const Polygon2d poly2({{0, 1}, {1, 0}, {0, -1}, {-1, 0}});
  const Polygon2d poly3(Box2d::CreateAABox({2, 2}, {3, 3}));
  const Polygon2d poly4(Box2d::CreateAABox({-10, -10}, {10, 10}));

  EXPECT_NEAR(poly1.DistanceTo(poly2), 0.0, 1e-5);
  EXPECT_NEAR(poly1.DistanceTo(poly3), sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly2.DistanceTo(poly3), 1.5 * sqrt(2.0), 1e-5);
  EXPECT_NEAR(poly4.DistanceTo(poly1), 0.0, 1e-5);
  EXPECT_NEAR(poly4.DistanceTo(poly2), 0.0, 1e-5);
  EXPECT_NEAR(poly4.DistanceTo(poly3), 0.0, 1e-5);
}

TEST(Polygon2dTest, ContainPolygon) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {3, 3}));
  const Polygon2d poly2(Box2d::CreateAABox({1, 1}, {2, 2}));
  const Polygon2d poly3(Box2d::CreateAABox({1.5, 1.5}, {4, 4}));
  const Polygon2d poly4(Box2d::CreateAABox({-10, -10}, {10, 10}));
  EXPECT_TRUE(poly1.Contains(poly2));
  EXPECT_FALSE(poly2.Contains(poly1));

  EXPECT_FALSE(poly1.Contains(poly3));
  EXPECT_FALSE(poly2.Contains(poly3));
  EXPECT_FALSE(poly3.Contains(poly1));
  EXPECT_FALSE(poly3.Contains(poly2));

  EXPECT_TRUE(poly4.Contains(poly1));
  EXPECT_TRUE(poly4.Contains(poly2));
  EXPECT_TRUE(poly4.Contains(poly3));

  const Polygon2d poly5(
      {{0, 0}, {4, 0}, {4, 2}, {3, 2}, {2, 1}, {1, 2}, {0, 2}});
  const Polygon2d poly6({{0, 1}, {4, 1}, {4, 2}, {0, 2}});
  const Polygon2d poly7({{0, 1}, {1, 1}, {1, 2}, {0, 2}});
  const Polygon2d poly8({{3, 1}, {4, 1}, {4, 2}, {3, 2}});
  const Polygon2d poly9({{0, 0}, {4, 0}, {4, 1}, {0, 1}});
  EXPECT_FALSE(poly5.Contains(poly6));
  EXPECT_TRUE(poly5.Contains(poly7));
  EXPECT_TRUE(poly5.Contains(poly8));
  EXPECT_TRUE(poly5.Contains(poly9));
}

TEST(Polygon2dTest, ConvexHull) {
  Polygon2d polygon;
  EXPECT_FALSE(Polygon2d::ComputeConvexHull({}, &polygon));
  EXPECT_FALSE(Polygon2d::ComputeConvexHull({{1, 2}}, &polygon));
  EXPECT_FALSE(Polygon2d::ComputeConvexHull({{3, 4}, {5, 6}}, &polygon));
  EXPECT_FALSE(
      Polygon2d::ComputeConvexHull({{3, 4}, {3, 4}, {5, 6}, {5, 6}}, &polygon));

  EXPECT_TRUE(Polygon2d::ComputeConvexHull({{0, 0}, {0, 4}, {3, 0}}, &polygon));
  EXPECT_TRUE(polygon.is_convex());
  EXPECT_NEAR(6.0, polygon.area(), 1e-5);

  EXPECT_TRUE(
      Polygon2d::ComputeConvexHull({{0, 0}, {0, 4}, {3, 0}, {3, 4}}, &polygon));
  EXPECT_TRUE(polygon.is_convex());
  EXPECT_NEAR(12.0, polygon.area(), 1e-5);

  EXPECT_TRUE(Polygon2d::ComputeConvexHull(
      {{0, 0}, {2, 2}, {1, 1}, {0, 4}, {3, 0}, {3, 4}}, &polygon));
  EXPECT_TRUE(polygon.is_convex());
  EXPECT_NEAR(12.0, polygon.area(), 1e-5);

  EXPECT_TRUE(Polygon2d::ComputeConvexHull(
      {{0, 0}, {0, 4}, {0, 1}, {0, 3}, {0, 2}, {1, 0}, {3, 0}, {2, 0}},
      &polygon));
  EXPECT_TRUE(polygon.is_convex());
  EXPECT_NEAR(6.0, polygon.area(), 1e-5);

  for (int iter = 0; iter < 10000; ++iter) {
    const int kRange = 10;
    const int n = RandomInt(3, 10);
    std::vector<Vec2d> points;
    for (int i = 0; i < n; ++i) {
      points.emplace_back(RandomInt(0, kRange), RandomInt(0, kRange));
    }
    double area = 0;
    for (int x0 = 0; x0 < kRange; ++x0) {
      double min_y = 0.0;
      double max_y = 0.0;
      if (ProjectByXSlow(points, x0 + 0.5, &min_y, &max_y)) {
        area += max_y - min_y;
      }
    }
    Polygon2d polygon;
    if (area < 1e-3) {
      EXPECT_FALSE(Polygon2d::ComputeConvexHull(points, &polygon));
    } else {
      EXPECT_TRUE(Polygon2d::ComputeConvexHull(points, &polygon));
      EXPECT_NEAR(area, polygon.area(), 1e-5);
    }
  }
}

TEST(Polygon2dTest, Overlap) {
  const Polygon2d poly1(Box2d::CreateAABox({0, 0}, {2, 2}));
  const Polygon2d poly2(Box2d::CreateAABox({1, 1}, {3, 3}));
  const Polygon2d poly3(Box2d::CreateAABox({2, 0}, {4, 2}));
  const Polygon2d poly4(Box2d({2, 2}, M_PI_4, sqrt(2.0), sqrt(2.0)));
  Polygon2d overlap_polygon;

  EXPECT_TRUE(poly1.ComputeOverlap(poly2, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.0, 1e-5);
  EXPECT_TRUE(poly2.ComputeOverlap(poly1, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.0, 1e-5);

  EXPECT_TRUE(poly2.ComputeOverlap(poly3, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.0, 1e-5);
  EXPECT_TRUE(poly3.ComputeOverlap(poly2, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.0, 1e-5);

  EXPECT_FALSE(poly1.ComputeOverlap(poly3, &overlap_polygon));
  EXPECT_FALSE(poly3.ComputeOverlap(poly1, &overlap_polygon));

  EXPECT_TRUE(poly1.ComputeOverlap(poly4, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 0.5, 1e-5);
  EXPECT_TRUE(poly4.ComputeOverlap(poly1, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 0.5, 1e-5);

  EXPECT_TRUE(poly2.ComputeOverlap(poly4, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 2.0, 1e-5);
  EXPECT_TRUE(poly4.ComputeOverlap(poly2, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 2.0, 1e-5);

  EXPECT_TRUE(poly3.ComputeOverlap(poly4, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 0.5, 1e-5);
  EXPECT_TRUE(poly4.ComputeOverlap(poly3, &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 0.5, 1e-5);

  EXPECT_NEAR(poly1.ComputeIoU(poly2), 0.1428, 1e-4);
  EXPECT_NEAR(poly2.ComputeIoU(poly1), 0.1428, 1e-4);

  EXPECT_NEAR(poly1.ComputeIoU(poly3), 0.0, 1e-4);
  EXPECT_NEAR(poly3.ComputeIoU(poly1), 0.0, 1e-4);

  EXPECT_NEAR(poly1.ComputeIoU(poly4), 0.0909, 1e-4);
  EXPECT_NEAR(poly4.ComputeIoU(poly1), 0.0909, 1e-4);

  EXPECT_NEAR(poly2.ComputeIoU(poly3), 0.1428, 1e-4);
  EXPECT_NEAR(poly3.ComputeIoU(poly2), 0.1428, 1e-4);

  EXPECT_NEAR(poly2.ComputeIoU(poly4), 0.5, 1e-4);
  EXPECT_NEAR(poly4.ComputeIoU(poly2), 0.5, 1e-4);

  EXPECT_NEAR(poly3.ComputeIoU(poly4), 0.0909, 1e-4);
  EXPECT_NEAR(poly4.ComputeIoU(poly3), 0.0909, 1e-4);

  Vec2d first_intersect;
  Vec2d last_intersect;
  EXPECT_FALSE(poly1.GetOverlap(LineSegment2d({-1, 0}, {-1, 2}),
                                &first_intersect, &last_intersect));
  EXPECT_FALSE(poly1.GetOverlap(LineSegment2d({-1, 1}, {-3, 1}),
                                &first_intersect, &last_intersect));
  EXPECT_FALSE(poly1.GetOverlap(LineSegment2d({1, 3}, {1, 5}), &first_intersect,
                                &last_intersect));

  EXPECT_TRUE(poly1.GetOverlap(LineSegment2d({1, -1}, {1, 3}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(1.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(0.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(2.0, last_intersect.y(), 1e-5);

  EXPECT_TRUE(poly1.GetOverlap(LineSegment2d({1, 1}, {1, 3}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(1.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(1.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(2.0, last_intersect.y(), 1e-5);

  EXPECT_TRUE(poly1.GetOverlap(LineSegment2d({1, -1}, {1, 1}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(1.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(0.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.y(), 1e-5);

  EXPECT_TRUE(poly1.GetOverlap(LineSegment2d({1, 3}, {3, 1}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(2.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(2.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(2.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(2.0, last_intersect.y(), 1e-5);

  EXPECT_FALSE(poly1.GetOverlap(LineSegment2d({4, 3}, {4, 3}), &first_intersect,
                                &last_intersect));
  EXPECT_TRUE(poly1.GetOverlap(LineSegment2d({1, 1}, {1, 1}), &first_intersect,
                               &last_intersect));
  EXPECT_NEAR(1.0, first_intersect.x(), 1e-5);
  EXPECT_NEAR(1.0, first_intersect.y(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.x(), 1e-5);
  EXPECT_NEAR(1.0, last_intersect.y(), 1e-5);

  const Polygon2d poly5(
      {{0, 0}, {4, 0}, {4, 2}, {3, 2}, {2, 1}, {1, 2}, {0, 2}});
  std::vector<LineSegment2d> overlap_line_segments =
      poly5.GetAllOverlaps(LineSegment2d({-10, 1.5}, {10, 1.5}));
  EXPECT_EQ(2, overlap_line_segments.size());
  EXPECT_NEAR(0.0, overlap_line_segments[0].start().x(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[0].start().y(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[0].end().x(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[0].end().y(), 1e-5);
  EXPECT_NEAR(2.5, overlap_line_segments[1].start().x(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[1].start().y(), 1e-5);
  EXPECT_NEAR(4.0, overlap_line_segments[1].end().x(), 1e-5);
  EXPECT_NEAR(1.5, overlap_line_segments[1].end().y(), 1e-5);

  overlap_line_segments =
      poly5.GetAllOverlaps(LineSegment2d({-10, 1}, {10, 1}));
  EXPECT_EQ(1, overlap_line_segments.size());
  EXPECT_NEAR(0.0, overlap_line_segments[0].start().x(), 1e-5);
  EXPECT_NEAR(1.0, overlap_line_segments[0].start().y(), 1e-5);
  EXPECT_NEAR(4.0, overlap_line_segments[0].end().x(), 1e-5);
  EXPECT_NEAR(1.0, overlap_line_segments[0].end().y(), 1e-5);

  overlap_line_segments =
      poly5.GetAllOverlaps(LineSegment2d({-10, 0.5}, {10, 0.5}));
  EXPECT_EQ(1, overlap_line_segments.size());
  EXPECT_NEAR(0.0, overlap_line_segments[0].start().x(), 1e-5);
  EXPECT_NEAR(0.5, overlap_line_segments[0].start().y(), 1e-5);
  EXPECT_NEAR(4.0, overlap_line_segments[0].end().x(), 1e-5);
  EXPECT_NEAR(0.5, overlap_line_segments[0].end().y(), 1e-5);

  overlap_line_segments =
      poly5.GetAllOverlaps(LineSegment2d({-10, -0.5}, {10, -0.5}));
  EXPECT_EQ(0, overlap_line_segments.size());
  overlap_line_segments =
      poly5.GetAllOverlaps(LineSegment2d({-10, 2.5}, {10, 2.5}));
  EXPECT_EQ(0, overlap_line_segments.size());

  overlap_line_segments =
      poly5.GetAllOverlaps(LineSegment2d({2, 0.5}, {2, 0.5}));
  EXPECT_EQ(1, overlap_line_segments.size());
  EXPECT_NEAR(2.0, overlap_line_segments[0].start().x(), 1e-5);
  EXPECT_NEAR(0.5, overlap_line_segments[0].start().y(), 1e-5);
  EXPECT_NEAR(2.0, overlap_line_segments[0].end().x(), 1e-5);
  EXPECT_NEAR(0.5, overlap_line_segments[0].end().y(), 1e-5);
  overlap_line_segments =
      poly5.GetAllOverlaps(LineSegment2d({5, 0.5}, {5, 0.5}));
  EXPECT_EQ(0, overlap_line_segments.size());

  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{1, 1}, {1, 3}, {3, 1}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 2.0, 1e-5);

  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{1, 1}, {-1, 1}, {1, 3}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 1.5, 1e-5);
  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{2, 1}, {-1, 1}, {2, 4}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 3.0, 1e-5);
  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{3, 1}, {-1, 1}, {3, 5}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 3.5, 1e-5);
  EXPECT_TRUE(Polygon2d({{0, 0}, {0, 4}, {4, 0}})
                  .ComputeOverlap(Polygon2d({{4, 1}, {-1, 1}, {4, 6}}),
                                  &overlap_polygon));
  EXPECT_NEAR(overlap_polygon.area(), 3.5, 1e-5);

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
    Box2d box1({x1, y1}, heading1, l1, w1);
    Box2d box2({x2, y2}, heading2, l2, w2);
    Box2d shrinked_box2({x2, y2}, heading2, l2 - 0.2, w2 - 0.2);
    Box2d extended_box2({x2, y2}, heading2, l2 + 0.2, w2 + 0.2);
    if (!box1.HasOverlap(extended_box2)) {
      EXPECT_FALSE(
          Polygon2d(box1).ComputeOverlap(Polygon2d(box2), &overlap_polygon));
    } else if (box1.HasOverlap(shrinked_box2)) {
      EXPECT_TRUE(
          Polygon2d(box1).ComputeOverlap(Polygon2d(box2), &overlap_polygon));
    }
  }

  for (int iter = 0; iter < 10000; ++iter) {
    const int kRange = 10;
    const int n1 = RandomInt(3, 10);
    const int n2 = RandomInt(3, 10);
    std::vector<Vec2d> points1, points2;
    for (int i = 0; i < n1; ++i) {
      points1.emplace_back(RandomInt(0, kRange), RandomInt(0, kRange));
    }
    for (int i = 0; i < n2; ++i) {
      points2.emplace_back(RandomInt(0, kRange), RandomInt(0, kRange));
    }
    Polygon2d polygon1;
    Polygon2d polygon2;
    if (!Polygon2d::ComputeConvexHull(points1, &polygon1) ||
        !Polygon2d::ComputeConvexHull(points2, &polygon2)) {
      continue;
    }
    std::vector<double> key_points;
    for (int x0 = 0; x0 <= kRange; ++x0) {
      key_points.push_back(x0);
    }
    for (const auto &line_segment1 : polygon1.line_segments()) {
      for (const auto &line_segment2 : polygon2.line_segments()) {
        Vec2d pt;
        if (line_segment1.GetIntersect(line_segment2, &pt)) {
          key_points.push_back(pt.x());
        }
      }
    }
    double area = 0;
    std::sort(key_points.begin(), key_points.end());
    for (size_t i = 0; i + 1 < key_points.size(); ++i) {
      const double width = key_points[i + 1] - key_points[i];
      if (width < 1e-6) {
        continue;
      }
      const double x = (key_points[i] + key_points[i + 1]) / 2.0;
      double min_y1 = 0.0;
      double max_y1 = 0.0;
      double min_y2 = 0.0;
      double max_y2 = 0.0;
      if (ProjectByXSlow(points1, x, &min_y1, &max_y1) &&
          ProjectByXSlow(points2, x, &min_y2, &max_y2)) {
        area +=
            std::max(0.0, std::min(max_y1, max_y2) - std::max(min_y1, min_y2)) *
            width;
      }
    }
    Polygon2d overlap_polygon;
    if (area < 1e-3) {
      EXPECT_FALSE(polygon1.ComputeOverlap(polygon2, &overlap_polygon));
    } else {
      EXPECT_TRUE(Polygon2d::ComputeConvexHull(points1, &polygon1));
      EXPECT_TRUE(Polygon2d::ComputeConvexHull(points2, &polygon2));
      EXPECT_TRUE(polygon1.ComputeOverlap(polygon2, &overlap_polygon));
      EXPECT_NEAR(area, overlap_polygon.area(), 1e-5);
    }
  }
}

TEST(Polygon2dTest, BoundingBox) {
  Polygon2d poly1(Box2d::CreateAABox({0, 0}, {2, 2}));
  Box2d box = poly1.BoundingBoxWithHeading(0.0);
  EXPECT_NEAR(1.0, box.center().x(), 1e-5);
  EXPECT_NEAR(1.0, box.center().y(), 1e-5);
  EXPECT_NEAR(4.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly1));
  AABox2d aabox = poly1.AABoundingBox();
  EXPECT_NEAR(1.0, aabox.center().x(), 1e-5);
  EXPECT_NEAR(1.0, aabox.center().y(), 1e-5);
  EXPECT_NEAR(4.0, aabox.area(), 1e-5);
  EXPECT_NEAR(2.0, aabox.length(), 1e-5);
  EXPECT_NEAR(2.0, aabox.width(), 1e-5);

  box = poly1.BoundingBoxWithHeading(M_PI_4);
  EXPECT_NEAR(1.0, box.center().x(), 1e-5);
  EXPECT_NEAR(1.0, box.center().y(), 1e-5);
  EXPECT_NEAR(8.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly1));

  box = poly1.MinAreaBoundingBox();
  EXPECT_NEAR(1.0, box.center().x(), 1e-5);
  EXPECT_NEAR(1.0, box.center().y(), 1e-5);
  EXPECT_NEAR(4.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly1));

  Polygon2d poly2({{1, 0}, {0, 1}, {-1, 0}, {0, -1}});
  box = poly2.BoundingBoxWithHeading(0.0);
  EXPECT_NEAR(0.0, box.center().x(), 1e-5);
  EXPECT_NEAR(0.0, box.center().y(), 1e-5);
  EXPECT_NEAR(4.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly2));
  aabox = poly2.AABoundingBox();
  EXPECT_NEAR(0.0, aabox.center().x(), 1e-5);
  EXPECT_NEAR(0.0, aabox.center().y(), 1e-5);
  EXPECT_NEAR(4.0, aabox.area(), 1e-5);
  EXPECT_NEAR(2.0, aabox.length(), 1e-5);
  EXPECT_NEAR(2.0, aabox.width(), 1e-5);

  box = poly2.BoundingBoxWithHeading(M_PI_4);
  EXPECT_NEAR(0.0, box.center().x(), 1e-5);
  EXPECT_NEAR(0.0, box.center().y(), 1e-5);
  EXPECT_NEAR(2.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly2));

  box = poly2.MinAreaBoundingBox();
  EXPECT_NEAR(0.0, box.center().x(), 1e-5);
  EXPECT_NEAR(0.0, box.center().y(), 1e-5);
  EXPECT_NEAR(2.0, box.area(), 1e-5);
  EXPECT_TRUE(Polygon2d(box).Contains(poly2));

  for (int iter = 0; iter < 1000; ++iter) {
    const int num_sample_points = RandomInt(3, 10);
    std::vector<Vec2d> points;
    for (int i = 0; i < num_sample_points; ++i) {
      const double x = RandomInt(-10, 10);
      const double y = RandomInt(-10, 10);
      points.emplace_back(x, y);
    }
    Polygon2d polygon;
    if (!Polygon2d::ComputeConvexHull(points, &polygon)) {
      continue;
    }
    double min_area = std::numeric_limits<double>::infinity();
    for (int iter2 = 0; iter2 < 10; ++iter2) {
      const double heading = RandomDouble(0, M_PI * 2.0);
      box = polygon.BoundingBoxWithHeading(heading);
      EXPECT_TRUE(Polygon2d(box).Contains(polygon));
      min_area = std::min(min_area, box.area());
    }
    box = polygon.MinAreaBoundingBox();
    EXPECT_TRUE(Polygon2d(box).Contains(polygon));
    EXPECT_LE(box.area(), min_area + 1e-5);
  }
}

TEST(Polygon2dTest, Expand) {
  {
    const Polygon2d poly(Box2d::CreateAABox({0, 0}, {2, 2}));
    const Polygon2d exp_poly = poly.ExpandByDistance(1.0);
    EXPECT_TRUE(exp_poly.is_convex());
    const Box2d box = exp_poly.BoundingBoxWithHeading(0.0);
    EXPECT_NEAR(box.center().x(), 1.0, 1e-6);
    EXPECT_NEAR(box.center().y(), 1.0, 1e-6);
    EXPECT_NEAR(box.width(), 4.0, 1e-6);
    EXPECT_NEAR(box.length(), 4.0, 1e-6);
    EXPECT_NEAR(exp_poly.area(), 12 + M_PI, 0.1);
  }
  {
    const std::vector<Vec2d> points{{0, 0}, {2, 0}, {2, 2}, {0, 2}, {1, 1}};
    const Polygon2d poly(points);
    const Polygon2d exp_poly = poly.ExpandByDistance(1.0);
    EXPECT_TRUE(exp_poly.is_convex());
    const Box2d box = exp_poly.BoundingBoxWithHeading(0.0);
    EXPECT_NEAR(box.center().x(), 1.0, 1e-6);
    EXPECT_NEAR(box.center().y(), 1.0, 1e-6);
    EXPECT_NEAR(box.width(), 4.0, 1e-6);
    EXPECT_NEAR(box.length(), 4.0, 1e-6);
    EXPECT_NEAR(exp_poly.area(), 12 + M_PI, 0.1);
  }
}

}  // namespace math
}  // namespace common
}  // namespace apollo
