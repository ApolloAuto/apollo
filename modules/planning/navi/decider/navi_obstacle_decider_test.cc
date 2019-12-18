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

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/planning/common/planning_gflags.h"

using apollo::common::util::PointFactory;
using apollo::perception::PerceptionObstacle;
using apollo::prediction::ObstaclePriority;

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
  common::VehicleState vehicle_state;

  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(2.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  Obstacle b1("1", perception_obstacle, ObstaclePriority::NORMAL, false);

  vec_points.push_back(PointFactory::ToPathPoint(0.0, 0.0));
  vec_points.push_back(PointFactory::ToPathPoint(0.0, 3.0, 0.0, 3.0));
  vec_obstacle.emplace_back(&b1);
  obstacle_boundary.set_start_l(1.5);

  b1.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddObstacle(b1);

  vehicle_state.set_linear_velocity(5.556);
  int lane_obstacles_num = 0;
  obstacle_decider.SetLastNudgeDistance(2);
  double nudge_dist;
  for (int i = 0; i < 6; i++) {
    lane_obstacles_num = 0;
    nudge_dist = obstacle_decider.GetNudgeDistance(
        vec_obstacle, reference_line, path_decision, vec_points, vehicle_state,
        &lane_obstacles_num);
  }
  EXPECT_FLOAT_EQ(nudge_dist, 0.755);
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
  common::VehicleState vehicle_state;

  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(-2.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  Obstacle b1("1", perception_obstacle, ObstaclePriority::NORMAL, false);
  b1.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddObstacle(b1);

  vec_points.push_back(PointFactory::ToPathPoint(0.0, 0.0));
  vec_points.push_back(PointFactory::ToPathPoint(0.0, 3.0, 0.0, 3.0));
  vec_obstacle.emplace_back(&b1);
  vehicle_state.set_linear_velocity(5.556);

  int lane_obstacles_num = 0;
  obstacle_decider.SetLastNudgeDistance(-2);
  double nudge_dist;
  for (int i = 0; i < 6; i++) {
    lane_obstacles_num = 0;
    nudge_dist = obstacle_decider.GetNudgeDistance(
        vec_obstacle, reference_line, path_decision, vec_points, vehicle_state,
        &lane_obstacles_num);
  }
  EXPECT_FLOAT_EQ(nudge_dist, -0.755);
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
  common::VehicleState vehicle_state;

  // obstacle 1
  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(3.0);
  perception_obstacle.mutable_position()->set_y(0.0);
  Obstacle b1("1", perception_obstacle, ObstaclePriority::NORMAL, false);
  obstacle_boundary.set_start_l(1.6);
  b1.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddObstacle(b1);

  // obstacle 2
  perception_obstacle.set_width(2.6);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(4.0);
  perception_obstacle.mutable_position()->set_y(0.0);
  Obstacle b2("2", perception_obstacle, ObstaclePriority::NORMAL, false);
  obstacle_boundary.set_start_l(1.5);
  b2.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddObstacle(b2);

  const double s1 = 0;
  const double s2 = s1 + std::sqrt(1.0 + 1.0);
  const double s3 = s2 + std::sqrt(1.0 + 1.0);
  const double s4 = s3 + std::sqrt(1.0 + 1.0);
  const double s5 = s4 + std::sqrt(1.0 + 1.0);
  vec_points.push_back(PointFactory::ToPathPoint(0.0, 0.0, 0.0, s1));
  vec_points.push_back(PointFactory::ToPathPoint(1.0, 1.0, 0.0, s2));
  vec_points.push_back(PointFactory::ToPathPoint(2.0, 2.0, 0.0, s3));
  vec_points.push_back(PointFactory::ToPathPoint(3.0, 3.0, 0.0, s4));
  vec_points.push_back(PointFactory::ToPathPoint(4.0, 4.0, 0.0, s5));
  vec_obstacle.emplace_back(&b1);
  vec_obstacle.emplace_back(&b2);
  vehicle_state.set_linear_velocity(5.556);

  int lane_obstacles_num = 0;
  obstacle_decider.SetLastNudgeDistance(2);
  double nudge_dist;
  for (int i = 0; i < 6; i++) {
    lane_obstacles_num = 0;
    nudge_dist = obstacle_decider.GetNudgeDistance(
        vec_obstacle, reference_line, path_decision, vec_points, vehicle_state,
        &lane_obstacles_num);
  }
  EXPECT_FLOAT_EQ(nudge_dist, 0.72657289);
  EXPECT_FLOAT_EQ(lane_obstacles_num, 2);
}

TEST(NaviObstacleDeciderTest, ComputeNudgeDist4) {
  NaviObstacleDecider obstacle_decider;
  std::vector<const Obstacle*> vec_obstacle;
  std::vector<common::PathPoint> vec_points;
  PerceptionObstacle perception_obstacle;
  PathDecision path_decision;
  SLBoundary obstacle_boundary;
  ReferenceLine reference_line;
  common::VehicleState vehicle_state;

  perception_obstacle.set_width(1.0);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(-3.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  Obstacle b1("1", perception_obstacle, ObstaclePriority::NORMAL, false);

  b1.SetPerceptionSlBoundary(obstacle_boundary);
  path_decision.AddObstacle(b1);

  vec_points.push_back(PointFactory::ToPathPoint(0.0, 0.0));
  vec_points.push_back(PointFactory::ToPathPoint(0.0, 3.0, 0.0, 3.0));
  vec_obstacle.emplace_back(&b1);
  vehicle_state.set_linear_velocity(20);

  int lane_obstacles_num = 0;
  double nudge_dist;
  for (int i = 0; i < 6; i++) {
    lane_obstacles_num = 0;
    nudge_dist = obstacle_decider.GetNudgeDistance(
        vec_obstacle, reference_line, path_decision, vec_points, vehicle_state,
        &lane_obstacles_num);
  }
  EXPECT_FLOAT_EQ(nudge_dist, 0);
  EXPECT_FLOAT_EQ(lane_obstacles_num, 0);
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
  Obstacle b1("5", perception_obstacle, ObstaclePriority::NORMAL, false);

  // obstacle 2
  perception_obstacle.set_width(2.6);
  perception_obstacle.set_length(1.0);
  perception_obstacle.mutable_position()->set_x(1.0);
  perception_obstacle.mutable_position()->set_y(-1.0);
  perception_obstacle.mutable_velocity()->set_x(5.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  Obstacle b2("6", perception_obstacle, ObstaclePriority::NORMAL, false);

  const double s1 = 0.0;
  const double s2 = s1 + 1.0;
  const double s3 = s2 + 1.0;
  const double s4 = s3 + 1.0;
  const double s5 = s4 + 1.0;
  vec_points.push_back(PointFactory::ToPathPoint(0.0, 2.0, 0.0, s1));
  vec_points.push_back(PointFactory::ToPathPoint(1.0, 2.0, 0.0, s2));
  vec_points.push_back(PointFactory::ToPathPoint(2.0, 2.0, 0.0, s3));
  vec_points.push_back(PointFactory::ToPathPoint(3.0, 2.0, 0.0, s4));
  vec_points.push_back(PointFactory::ToPathPoint(4.0, 2.0, 0.0, s5));
  vec_obstacle.emplace_back(&b1);
  vec_obstacle.emplace_back(&b2);

  std::vector<std::tuple<std::string, double, double>> unsafe_obstacle_info;
  obstacle_decider.GetUnsafeObstaclesInfo(vec_points, vec_obstacle);
  unsafe_obstacle_info = obstacle_decider.UnsafeObstacles();
  EXPECT_EQ(std::get<0>(unsafe_obstacle_info[0]), "5");
  EXPECT_EQ(std::get<0>(unsafe_obstacle_info[1]), "6");
  EXPECT_EQ(std::get<1>(unsafe_obstacle_info[0]), 1.5);
  EXPECT_EQ(std::get<1>(unsafe_obstacle_info[1]), 0.5);
  EXPECT_EQ(std::get<2>(unsafe_obstacle_info[0]), 10);
  EXPECT_EQ(std::get<2>(unsafe_obstacle_info[1]), 5);
}

}  // namespace planning
}  // namespace apollo
