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
 * "NaviObstacleDecider".
 */

#include "modules/planning/navi/decider/navi_obstacle_decider.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/navi/common/local_path.h"

using apollo::common::PathPoint;
using apollo::common::util::MakePathPoint;
using apollo::perception::PerceptionObstacle;
using std::vector;

namespace apollo {
namespace planning {

TEST(NaviObstacleDeciderTest, ComputeNudgeDist1) {
  NaviObstacleDecider obstacle_decider;
  std::vector<const Obstacle*> vec_obstacle;
  std::vector<common::PathPoint> vec_points;
  PerceptionObstacle perception_obstacle;

  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(2.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  Obstacle b1("1", perception_obstacle);

  PathPoint p1 = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.set_s(0.0);
  PathPoint p2 = MakePathPoint(0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p2.set_s(3.0);
  vec_points.emplace_back(p1);
  vec_points.emplace_back(p2);
  vec_obstacle.emplace_back(&b1);
  NaviObstacleDecider navi_obstacle;

  double nudge_dist =
      obstacle_decider.GetNudgeDistance(vec_obstacle, vec_points, 3.3);
  EXPECT_FLOAT_EQ(nudge_dist, 0.455);
}

TEST(NaviObstacleDeciderTest, ComputeNudgeDist2) {
  NaviObstacleDecider obstacle_decider;
  std::vector<const Obstacle*> vec_obstacle;
  std::vector<common::PathPoint> vec_points;
  PerceptionObstacle perception_obstacle;

  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(-2.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  Obstacle b1("1", perception_obstacle);

  PathPoint p1 = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.set_s(0.0);
  PathPoint p2 = MakePathPoint(0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p2.set_s(3.0);
  vec_points.emplace_back(p1);
  vec_points.emplace_back(p2);
  vec_obstacle.emplace_back(&b1);
  NaviObstacleDecider navi_obstacle;

  double nudge_dist =
      obstacle_decider.GetNudgeDistance(vec_obstacle, vec_points, 3.3);
  EXPECT_FLOAT_EQ(nudge_dist, -0.595);
}

TEST(NaviObstacleDeciderTest, ComputeNudgeDist3) {
  NaviObstacleDecider obstacle_decider;
  std::vector<const Obstacle*> vec_obstacle;
  std::vector<common::PathPoint> vec_points;
  PerceptionObstacle perception_obstacle;

  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(3.0);
  perception_obstacle.mutable_position()->set_y(0.0);
  Obstacle b1("1", perception_obstacle);

  PathPoint p1 = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.set_s(0.0);
  PathPoint p2 = MakePathPoint(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p2.set_s(p1.s() + std::sqrt(1.0 + 1.0));
  PathPoint p3 = MakePathPoint(2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p3.set_s(p2.s() + std::sqrt(1.0 + 1.0));
  PathPoint p4 = MakePathPoint(3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p4.set_s(p3.s() + std::sqrt(1.0 + 1.0));
  vec_points.emplace_back(p1);
  vec_points.emplace_back(p2);
  vec_points.emplace_back(p3);
  vec_points.emplace_back(p4);
  vec_obstacle.emplace_back(&b1);
  NaviObstacleDecider navi_obstacle;

  double nudge_dist =
      obstacle_decider.GetNudgeDistance(vec_obstacle, vec_points, 3.3);
  EXPECT_FLOAT_EQ(nudge_dist, 0.33367965);
}
}  // namespace planning
}  // namespace apollo
