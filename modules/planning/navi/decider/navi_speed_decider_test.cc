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
 * "NaviSpeedDecider".
 */

#include "modules/planning/navi/decider/navi_speed_decider.h"

#include <map>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::util::MakePathPoint;
using apollo::perception::PerceptionObstacle;

namespace apollo {
namespace planning {

TEST(NaviSpeedDeciderTest, CreateSpeedData) {
  NaviSpeedDecider speed_decider;
  speed_decider.preferred_speed_ = 10.0;
  speed_decider.max_speed_ = 20.0;
  speed_decider.preferred_accel_ = 1.0;
  speed_decider.preferred_decel_ = 1.0;
  speed_decider.max_accel_ = 5.0;
  speed_decider.max_decel_ = 5.0;
  speed_decider.obstacle_buffer_ = 1.0;
  speed_decider.safe_distance_base_ = 2.0;
  speed_decider.safe_distance_ratio_ = 1.0;

  PerceptionObstacle perception_obstacle;
  std::map<std::string, Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  std::vector<PathPoint> path_data_points;
  path_data_points.emplace_back(
      MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  SpeedData speed_data;
  EXPECT_EQ(Status::OK(), speed_decider.MakeSpeedDecision(
                              0.0, 0.0, 0.0, 100.0, path_data_points, obstacles,
                              [&](const std::string& id) mutable {
                                return &obstacle_buf[id];
                              },
                              &speed_data));

  for (auto& p : speed_data.speed_vector()) {
    if (p.s() > 25.0 && p.s() < 85.0) EXPECT_NEAR(10.0, p.v(), 1.0);
    if (p.s() > 98.0) EXPECT_NEAR(0.0, p.v(), 0.01);
  }
}

TEST(NaviSpeedDeciderTest, CreateSpeedDataForStaticObstacle) {
  NaviSpeedDecider speed_decider;
  speed_decider.preferred_speed_ = 10.0;
  speed_decider.max_speed_ = 20.0;
  speed_decider.preferred_accel_ = 1.0;
  speed_decider.preferred_decel_ = 1.0;
  speed_decider.max_accel_ = 5.0;
  speed_decider.max_decel_ = 5.0;
  speed_decider.obstacle_buffer_ = 1.0;
  speed_decider.safe_distance_base_ = 2.0;
  speed_decider.safe_distance_ratio_ = 1.0;

  PerceptionObstacle perception_obstacle;
  std::map<std::string, Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  std::vector<PathPoint> path_data_points;
  path_data_points.emplace_back(
      MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  // obstacle1
  perception_obstacle.mutable_position()->set_x(50.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  perception_obstacle.mutable_velocity()->set_x(0.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  std::string id = "1";
  obstacle_buf.emplace(id, Obstacle(id, perception_obstacle));
  obstacles.emplace_back(&obstacle_buf[id]);

  SpeedData speed_data;
  EXPECT_EQ(Status::OK(), speed_decider.MakeSpeedDecision(
                              0.0, 0.0, 0.0, 100.0, path_data_points, obstacles,
                              [&](const std::string& id) mutable {
                                return &obstacle_buf[id];
                              },
                              &speed_data));
  for (auto& p : speed_data.speed_vector()) {
    if (p.s() > 25.0 && p.s() < 30.0) EXPECT_NEAR(10.0, p.v(), 1.0);
    if (p.s() > 41.8) EXPECT_NEAR(0.0, p.v(), 1.0);
  }
}

TEST(NaviSpeedDeciderTest, CreateSpeedDataForObstacles) {
  NaviSpeedDecider speed_decider;
  speed_decider.preferred_speed_ = 10.0;
  speed_decider.max_speed_ = 20.0;
  speed_decider.preferred_accel_ = 1.0;
  speed_decider.preferred_decel_ = 1.0;
  speed_decider.max_accel_ = 5.0;
  speed_decider.max_decel_ = 5.0;
  speed_decider.obstacle_buffer_ = 1.0;
  speed_decider.safe_distance_base_ = 2.0;
  speed_decider.safe_distance_ratio_ = 1.0;

  PerceptionObstacle perception_obstacle;
  std::map<std::string, Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  std::vector<PathPoint> path_data_points;
  path_data_points.emplace_back(
      MakePathPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  // obstacle1
  perception_obstacle.mutable_position()->set_x(50.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  perception_obstacle.mutable_velocity()->set_x(-10.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  std::string id = "1";
  obstacle_buf.emplace(id, Obstacle(id, perception_obstacle));
  obstacles.emplace_back(&obstacle_buf[id]);

  // obstacle2
  perception_obstacle.mutable_position()->set_x(20.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  perception_obstacle.mutable_velocity()->set_x(-5.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  id = "2";
  obstacle_buf.emplace(id, Obstacle(id, perception_obstacle));
  obstacles.emplace_back(&obstacle_buf[id]);

  SpeedData speed_data;
  EXPECT_EQ(
      Status::OK(),
      speed_decider.MakeSpeedDecision(
          10.0, 0.0, 0.0, 100.0, path_data_points, obstacles,
          [&](const std::string& id) mutable { return &obstacle_buf[id]; },
          &speed_data));
  for (auto& p : speed_data.speed_vector()) {
    if (p.s() < 5.0) EXPECT_NEAR(10.0, p.v(), 1.0);
    if (p.s() > 25.0 && p.s() < 35.0) EXPECT_NEAR(5.0, p.v(), 1.0);
    if (p.s() > 40.8) EXPECT_NEAR(0.0, p.v(), 1.0);
  }
}

TEST(NaviSpeedDeciderTest, ErrorTest) {}

}  // namespace planning
}  // namespace apollo
