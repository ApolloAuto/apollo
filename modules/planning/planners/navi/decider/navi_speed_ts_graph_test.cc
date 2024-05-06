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
 * @brief This file provides several unit tests for the class
 * "NaviSpeedTsGraph".
 */

#include "modules/planning/planners/navi/decider/navi_speed_ts_graph.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

TEST(NaviSpeedTsGraph, Solve1) {
  NaviSpeedTsGraph graph;
  graph.Reset(1.0, 100.0, 0.0, 0.0, 0.0);

  NaviSpeedTsConstraints constraints;
  constraints.v_max = 20.0;
  constraints.v_preffered = 10.0;
  constraints.a_max = 4.0;
  constraints.a_preffered = 2.0;
  constraints.b_max = 5.0;
  constraints.b_preffered = 2.0;
  graph.UpdateConstraints(constraints);

  std::vector<NaviSpeedTsPoint> points;
  EXPECT_EQ(Status::OK(), graph.Solve(&points));
  EXPECT_NEAR(0.0, points.front().s, 0.1);
  EXPECT_NEAR(0.0, points.front().t, 0.1);
  EXPECT_NEAR(0.0, points.front().v, 0.1);
  EXPECT_NEAR(25.0, points[25].s, 0.1);
  EXPECT_NEAR(5.0, points[25].t, 0.1);
  EXPECT_NEAR(10.0, points[25].v, 0.1);
  for (const auto& point : points)
    if (point.s > 25.0) {
      EXPECT_NEAR(10.0, point.v, 0.1);
    }
}

TEST(NaviSpeedTsGraph, Solve2) {
  NaviSpeedTsGraph graph;
  graph.Reset(1.0, 100.0, 0.0, 0.0, 0.0);
  auto get_safe_distance = [](double v) { return 1.0 * v + 2.0; };

  NaviSpeedTsConstraints constraints;
  constraints.v_max = 20.0;
  constraints.v_preffered = 10.0;
  constraints.a_max = 4.0;
  constraints.a_preffered = 2.0;
  constraints.b_max = 5.0;
  constraints.b_preffered = 2.0;
  graph.UpdateConstraints(constraints);

  graph.UpdateObstacleConstraints(40.0, get_safe_distance(0.0), 0.5, 0.0, 10.0);

  std::vector<NaviSpeedTsPoint> points;
  EXPECT_EQ(Status::OK(), graph.Solve(&points));
  EXPECT_NEAR(0.0, points.front().s, 0.1);
  EXPECT_NEAR(0.0, points.front().t, 0.1);
  EXPECT_NEAR(0.0, points.front().v, 0.1);
  for (const auto& point : points)
    if (point.s > 38.0) {
      EXPECT_NEAR(0.0, point.v, 0.1);
    }
  EXPECT_NEAR(0.0, points.back().v, 0.1);
}

TEST(NaviSpeedTsGraph, Solve3) {
  NaviSpeedTsGraph graph;
  graph.Reset(1.0, 100.0, 5.0, 0.0, 0.0);
  auto get_safe_distance = [](double v) { return 1.0 * v + 2.0; };

  NaviSpeedTsConstraints constraints;
  constraints.v_max = 20.0;
  constraints.v_preffered = 10.0;
  constraints.a_max = 4.0;
  constraints.a_preffered = 2.0;
  constraints.b_max = 5.0;
  constraints.b_preffered = 2.0;
  graph.UpdateConstraints(constraints);

  graph.UpdateObstacleConstraints(10.0, get_safe_distance(5.0), 0.5, 5.0, 10.0);

  std::vector<NaviSpeedTsPoint> points;
  EXPECT_EQ(Status::OK(), graph.Solve(&points));
  EXPECT_NEAR(0.0, points.front().s, 0.1);
  EXPECT_NEAR(0.0, points.front().t, 0.1);
  EXPECT_NEAR(5.0, points.front().v, 0.1);
  for (const auto& point : points) {
    if (point.s > 15.0) {
      auto obstacle_distance = 5.0 * point.t + 10.0;
      EXPECT_GE(obstacle_distance, point.s);
      EXPECT_NEAR(5.0, point.v, 0.1);
    }
  }
}

TEST(NaviSpeedTsGraph, ErrorTest) {}

}  // namespace planning
}  // namespace apollo
