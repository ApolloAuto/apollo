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
  PathDecision path_decision;
  SLBoundary obstacle_boundary;
  ReferenceLine reference_line;

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
  obstacle_boundary.set_start_l(1.5);
  PathObstacle path_obstacles(&b1);
  path_obstacles.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddPathObstacle(path_obstacles);

  int lane_obstacles_num = 0;
  double nudge_dist = obstacle_decider.GetNudgeDistance(
      vec_obstacle, reference_line, path_decision, vec_points,
      &lane_obstacles_num);
  EXPECT_FLOAT_EQ(nudge_dist, 0.455);
  EXPECT_FLOAT_EQ(lane_obstacles_num, 1);
}

TEST(NaviObstacleDeciderTest, ComputeNudgeDist2) {
  NaviObstacleDecider obstacle_decider;
  std::vector<const Obstacle*> vec_obstacle;
  std::vector<common::PathPoint> vec_points;
  PerceptionObstacle perception_obstacle;
  PathDecision path_decision;
  SLBoundary obstacle_boundary;
  ReferenceLine reference_line;

  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(-2.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  Obstacle b1("1", perception_obstacle);
  obstacle_boundary.set_start_l(1.7);
  PathObstacle path_obstacles(&b1);
  path_obstacles.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddPathObstacle(path_obstacles);

  PathPoint p1 = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.set_s(0.0);
  PathPoint p2 = MakePathPoint(0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p2.set_s(3.0);
  vec_points.emplace_back(p1);
  vec_points.emplace_back(p2);
  vec_obstacle.emplace_back(&b1);

  int lane_obstacles_num = 0;
  double nudge_dist = obstacle_decider.GetNudgeDistance(
      vec_obstacle, reference_line, path_decision, vec_points,
      &lane_obstacles_num);
  EXPECT_FLOAT_EQ(nudge_dist, -0.455);
  EXPECT_FLOAT_EQ(lane_obstacles_num, 1);
}

TEST(NaviObstacleDeciderTest, ComputeNudgeDist3) {
  NaviObstacleDecider obstacle_decider;
  std::vector<const Obstacle*> vec_obstacle;
  std::vector<common::PathPoint> vec_points;
  PerceptionObstacle perception_obstacle;
  PathDecision path_decision;
  SLBoundary obstacle_boundary;
  ReferenceLine reference_line;

  // obstacle 1
  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(3.0);
  perception_obstacle.mutable_position()->set_y(0.0);
  Obstacle b1("1", perception_obstacle);
  obstacle_boundary.set_start_l(1.6);
  PathObstacle path_obstacles1(&b1);
  path_obstacles1.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddPathObstacle(path_obstacles1);

  // obstacle 2
  perception_obstacle.set_width(2.6);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(4.0);
  perception_obstacle.mutable_position()->set_y(0.0);
  Obstacle b2("2", perception_obstacle);
  obstacle_boundary.set_start_l(1.5);
  PathObstacle path_obstacles2(&b2);
  path_obstacles2.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddPathObstacle(path_obstacles2);

  PathPoint p1 = MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.set_s(0.0);
  PathPoint p2 = MakePathPoint(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p2.set_s(p1.s() + std::sqrt(1.0 + 1.0));
  PathPoint p3 = MakePathPoint(2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p3.set_s(p2.s() + std::sqrt(1.0 + 1.0));
  PathPoint p4 = MakePathPoint(3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p4.set_s(p3.s() + std::sqrt(1.0 + 1.0));
  PathPoint p5 = MakePathPoint(4.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p5.set_s(p4.s() + std::sqrt(1.0 + 1.0));
  vec_points.emplace_back(p1);
  vec_points.emplace_back(p2);
  vec_points.emplace_back(p3);
  vec_points.emplace_back(p4);
  vec_points.emplace_back(p5);
  vec_obstacle.emplace_back(&b1);
  vec_obstacle.emplace_back(&b2);

  int lane_obstacles_num = 0;
  double nudge_dist = obstacle_decider.GetNudgeDistance(
      vec_obstacle, reference_line, path_decision, vec_points,
      &lane_obstacles_num);
  EXPECT_FLOAT_EQ(nudge_dist, 0.42657289);
  EXPECT_FLOAT_EQ(lane_obstacles_num, 2);
}

TEST(NaviObstacleDeciderTest, GetUnsafeObstaclesID) {
  NaviObstacleDecider obstacle_decider;
  std::vector<const Obstacle*> vec_obstacle;
  std::vector<common::PathPoint> vec_points;
  PerceptionObstacle perception_obstacle;

  // obstacle 1
  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(2.0);
  perception_obstacle.mutable_position()->set_y(3.0);
  perception_obstacle.mutable_velocity()->set_x(10.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  Obstacle b1("5", perception_obstacle);

  // obstacle 2
  perception_obstacle.set_width(2.6);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(1.0);
  perception_obstacle.mutable_position()->set_y(-1.0);
  perception_obstacle.mutable_velocity()->set_x(5.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  Obstacle b2("6", perception_obstacle);

  PathPoint p1 = MakePathPoint(0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.set_s(0.0);
  PathPoint p2 = MakePathPoint(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p2.set_s(p1.s() + 1.0);
  PathPoint p3 = MakePathPoint(2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p3.set_s(p2.s() + 1.0);
  PathPoint p4 = MakePathPoint(3.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p4.set_s(p3.s() + 1.0);
  PathPoint p5 = MakePathPoint(4.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p5.set_s(p4.s() + 1.0);
  vec_points.emplace_back(p1);
  vec_points.emplace_back(p2);
  vec_points.emplace_back(p3);
  vec_points.emplace_back(p4);
  vec_points.emplace_back(p5);
  vec_obstacle.emplace_back(&b1);
  vec_obstacle.emplace_back(&b2);

  std::vector<std::tuple<std::string, double, double>> unsafe_obstacle_info;
  obstacle_decider.GetUnsafeObstaclesInfo(vec_points, vec_obstacle);
  unsafe_obstacle_info = obstacle_decider.UnsafeObstacles();
  EXPECT_EQ(std::get<0>(unsafe_obstacle_info[0]), "5");
  EXPECT_EQ(std::get<0>(unsafe_obstacle_info[1]), "6");
  EXPECT_EQ(std::get<1>(unsafe_obstacle_info[0]), 2);
  EXPECT_EQ(std::get<1>(unsafe_obstacle_info[1]), 1);
  EXPECT_EQ(std::get<2>(unsafe_obstacle_info[0]), 10);
  EXPECT_EQ(std::get<2>(unsafe_obstacle_info[1]), 5);
}
}  // namespace planning
}  // namespace apollo
