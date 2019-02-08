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

#include "modules/planning/common/speed/st_boundary.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

TEST(StBoundaryTest, basic_test) {
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  lower_points.emplace_back(0.0, 0.0);
  lower_points.emplace_back(0.0, 10.0);
  upper_points.emplace_back(5.0, 0.0);
  upper_points.emplace_back(5.0, 10.0);

  point_pairs.emplace_back(lower_points[0], upper_points[0]);
  point_pairs.emplace_back(lower_points[1], upper_points[1]);

  STBoundary boundary(point_pairs);

  EXPECT_EQ(boundary.id(), "");
  EXPECT_EQ(boundary.boundary_type(), STBoundary::BoundaryType::UNKNOWN);
  EXPECT_DOUBLE_EQ(0.0, boundary.min_s());
  EXPECT_DOUBLE_EQ(5.0, boundary.max_s());
  EXPECT_DOUBLE_EQ(0.0, boundary.min_t());
  EXPECT_DOUBLE_EQ(10.0, boundary.max_t());
}

TEST(StBoundaryTest, boundary_range) {
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  lower_points.emplace_back(1.0, 0.0);
  lower_points.emplace_back(1.0, 10.0);
  upper_points.emplace_back(5.0, 0.0);
  upper_points.emplace_back(5.0, 10.0);

  point_pairs.emplace_back(lower_points[0], upper_points[0]);
  point_pairs.emplace_back(lower_points[1], upper_points[1]);

  STBoundary boundary(point_pairs);

  boundary.SetBoundaryType(STBoundary::BoundaryType::YIELD);
  double t = -10.0;
  const double dt = 0.01;
  while (t < 10.0) {
    double low = 0.0;
    double high = 0.0;
    if (t < 0.0) {
      EXPECT_TRUE(boundary.GetUnblockSRange(t, &high, &low));
      EXPECT_DOUBLE_EQ(low, 0.0);
      EXPECT_DOUBLE_EQ(high, 200.0);
      EXPECT_FALSE(boundary.GetBoundarySRange(t, &high, &low));
    } else {
      EXPECT_TRUE(boundary.GetUnblockSRange(t, &high, &low));
      EXPECT_DOUBLE_EQ(low, 0.0);
      EXPECT_DOUBLE_EQ(high, 1.0);

      EXPECT_TRUE(boundary.GetBoundarySRange(t, &high, &low));
      EXPECT_DOUBLE_EQ(low, 1.0);
      EXPECT_DOUBLE_EQ(high, 5.0);
    }
    t += dt;
  }
}

TEST(StBoundaryTest, get_index_range) {
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  lower_points.emplace_back(43.000164837720789, -517957.08587679861);
  lower_points.emplace_back(46.100164825451913, -517955.58587660792);

  upper_points.emplace_back(52.200164801309178, -517957.08587679861);
  upper_points.emplace_back(55.6001647283625, -517955.58587660792);

  point_pairs.emplace_back(lower_points[0], upper_points[0]);
  point_pairs.emplace_back(lower_points[1], upper_points[1]);

  STBoundary boundary(point_pairs);

  size_t left = 0;
  size_t right = 0;

  EXPECT_TRUE(
      boundary.GetIndexRange(lower_points, -517957.08587679861, &left, &right));
  EXPECT_EQ(left, 0);
  EXPECT_EQ(right, 0);

  EXPECT_TRUE(
      boundary.GetIndexRange(lower_points, -517955.58587660792, &left, &right));
  EXPECT_EQ(left, 0);
  EXPECT_EQ(right, 1);

  EXPECT_TRUE(
      boundary.GetIndexRange(lower_points, -517955.58587660792, &left, &right));
  EXPECT_EQ(left, 0);
  EXPECT_EQ(right, 1);

  EXPECT_FALSE(boundary.GetIndexRange(lower_points, 0.0, &left, &right));
}

TEST(StBoundaryTest, remove_redundant_points) {
  std::vector<std::pair<STPoint, STPoint>> points;
  points.emplace_back(STPoint(0.0, 0.0), STPoint(1.0, 0.0));
  points.emplace_back(STPoint(0.1, 0.2), STPoint(1.1, 0.2));
  points.emplace_back(STPoint(0.2, 0.3), STPoint(1.2, 0.3));
  points.emplace_back(STPoint(0.3, 0.4), STPoint(1.3, 0.4));
  points.emplace_back(STPoint(0.4, 0.5), STPoint(1.4, 0.5));

  EXPECT_EQ(points.size(), 5);

  STBoundary st_boundary;
  st_boundary.RemoveRedundantPoints(&points);

  EXPECT_EQ(points.size(), 2);
  EXPECT_DOUBLE_EQ(points[0].first.s(), 0.0);
  EXPECT_DOUBLE_EQ(points[0].first.t(), 0.0);
  EXPECT_DOUBLE_EQ(points[0].second.s(), 1.0);
  EXPECT_DOUBLE_EQ(points[0].second.t(), 0.0);

  EXPECT_DOUBLE_EQ(points[1].first.s(), 0.4);
  EXPECT_DOUBLE_EQ(points[1].first.t(), 0.5);
  EXPECT_DOUBLE_EQ(points[1].second.s(), 1.4);
  EXPECT_DOUBLE_EQ(points[1].second.t(), 0.5);
}

}  // namespace planning
}  // namespace apollo
