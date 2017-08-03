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

#include "modules/planning/optimizer/st_graph/st_graph_boundary.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ros/include/ros/ros.h"

#include "modules/common/log.h"

namespace apollo {
namespace planning {

TEST(StGraphBoundaryTest, basic_test) {
  std::vector<STPoint> st_points;
  st_points.emplace_back(0.0, 0.0);
  st_points.emplace_back(0.0, 10.0);
  st_points.emplace_back(5.0, 10.0);
  st_points.emplace_back(5.0, 0.0);

  StGraphBoundary boundary(st_points);
  EXPECT_EQ(boundary.id(), "");
  EXPECT_EQ(boundary.boundary_type(), StGraphBoundary::BoundaryType::UNKNOWN);
  double left_t = 0.0;
  double right_t = 0.0;
  boundary.GetBoundaryTimeScope(&left_t, &right_t);
  EXPECT_DOUBLE_EQ(left_t, 0.0);
  EXPECT_DOUBLE_EQ(right_t, 10.0);
}

}  // namespace planning
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
