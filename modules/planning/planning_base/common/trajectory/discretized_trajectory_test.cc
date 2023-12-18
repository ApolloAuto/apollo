/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/planning_base/common/trajectory/discretized_trajectory.h"

#include "gtest/gtest.h"

#include "cyber/common/file.h"

namespace apollo {
namespace planning {

TEST(basic_test, DiscretizedTrajectory) {
  const std::string path_of_standard_trajectory =
      "modules/planning/planning_base/testdata/trajectory_data/"
      "standard_trajectory.pb.txt";
  ADCTrajectory trajectory;
  EXPECT_TRUE(cyber::common::GetProtoFromFile(path_of_standard_trajectory,
                                              &trajectory));
  DiscretizedTrajectory discretized_trajectory(trajectory);
  EXPECT_DOUBLE_EQ(discretized_trajectory.GetTemporalLength(),
                   7.9999999999999885);
  EXPECT_DOUBLE_EQ(discretized_trajectory.GetSpatialLength(),
                   44.752319202675167);
  auto p1 = discretized_trajectory.Evaluate(4.0);
  EXPECT_DOUBLE_EQ(p1.path_point().x(), 587263.01182131236);
  EXPECT_DOUBLE_EQ(p1.path_point().y(), 4140966.5720794979);
  EXPECT_DOUBLE_EQ(p1.relative_time(), 4.0);
  EXPECT_DOUBLE_EQ(p1.v(), 5.4412586837131443);

  auto k1 = discretized_trajectory.QueryLowerBoundPoint(2.12);
  EXPECT_EQ(k1, 62);

  auto k2 = discretized_trajectory.QueryNearestPoint({587264.0, 4140966.2});
  EXPECT_EQ(k2, 80);

  EXPECT_EQ(discretized_trajectory.NumOfPoints(), 121);
}

}  // namespace planning
}  // namespace apollo
