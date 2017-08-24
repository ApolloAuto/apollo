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

#include "modules/planning/tasks/st_graph/st_boundary.h"

#include <algorithm>
#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/common/log.h"

namespace apollo {
namespace planning {

TEST(StBoundaryTest, basic_test) {
  std::vector<STPoint> st_points;
  st_points.emplace_back(0.0, 0.0);
  st_points.emplace_back(0.0, 10.0);
  st_points.emplace_back(5.0, 10.0);
  st_points.emplace_back(5.0, 0.0);

  StBoundary boundary(st_points);
  EXPECT_EQ(boundary.id(), "");
  EXPECT_EQ(boundary.boundary_type(), StBoundary::BoundaryType::UNKNOWN);
  EXPECT_FLOAT_EQ(0.0, boundary.min_s());
  EXPECT_FLOAT_EQ(5.0, boundary.max_s());
  EXPECT_FLOAT_EQ(0.0, boundary.min_t());
  EXPECT_FLOAT_EQ(10.0, boundary.max_t());
}

TEST(StBoundaryTest, boundary_range) {
  std::vector<STPoint> st_points;
  st_points.emplace_back(1.0, 0.0);
  st_points.emplace_back(1.0, 10.0);
  st_points.emplace_back(5.0, 10.0);
  st_points.emplace_back(5.0, 0.0);
  StBoundary boundary(st_points);
  boundary.SetBoundaryType(StBoundary::BoundaryType::YIELD);
  double t = -10.0;
  const double dt = 0.01;
  while (t < 10.0) {
    double low = 0.0;
    double high = 0.0;
    if (t < -1e-6) {
      EXPECT_FALSE(boundary.GetUnblockSRange(t, &high, &low));
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

}  // namespace planning
}  // namespace apollo
