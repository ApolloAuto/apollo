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

#include "modules/dreamview/backend/util/trajectory_point_collector.h"

#include <cmath>

#include "gtest/gtest.h"

using apollo::common::TrajectoryPoint;

namespace apollo {
namespace dreamview {
namespace util {

TEST(TrajectoryPointCollectorTest, ThreePoints) {
  SimulationWorld world;
  TrajectoryPointCollector collector(&world);

  const double base_time = 1000.0;
  for (int i = 0; i < 4; ++i) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(i * 100.0);
    point.mutable_path_point()->set_y(i * 100.0 + 100.0);
    point.set_relative_time(i * 1000.0);
    collector.Collect(point, base_time);
  }

  EXPECT_EQ(world.planning_trajectory_size(), 3);

  {
    const Object &point = world.planning_trajectory(0);
    EXPECT_DOUBLE_EQ(0.0, point.position_x());
    EXPECT_DOUBLE_EQ(100.0, point.position_y());
    EXPECT_DOUBLE_EQ(1000.0, point.timestamp_sec());
    EXPECT_DOUBLE_EQ(atan2(100.0, 100.0), point.heading());
  }

  {
    const Object &point = world.planning_trajectory(1);
    EXPECT_DOUBLE_EQ(100.0, point.position_x());
    EXPECT_DOUBLE_EQ(200.0, point.position_y());
    EXPECT_DOUBLE_EQ(2000.0, point.timestamp_sec());
    EXPECT_DOUBLE_EQ(atan2(100.0, 100.0), point.heading());
  }

  {
    const Object &point = world.planning_trajectory(2);
    EXPECT_DOUBLE_EQ(200.0, point.position_x());
    EXPECT_DOUBLE_EQ(300.0, point.position_y());
    EXPECT_DOUBLE_EQ(3000.0, point.timestamp_sec());
    EXPECT_DOUBLE_EQ(atan2(100.0, 100.0), point.heading());
  }
}

}  // namespace util
}  // namespace dreamview
}  // namespace apollo
