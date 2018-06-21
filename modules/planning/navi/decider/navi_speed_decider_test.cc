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

#include <list>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

using apollo::common::Status;
using apollo::common::VehicleState;
using apollo::perception::PerceptionObstacle;

namespace apollo {
namespace planning {

TEST(NaviSpeedDeciderTest, CreateSpeedData1) {
  VehicleState vehicle_state;
  vehicle_state.set_linear_velocity(0.0);

  PerceptionObstacle perception_obstacle;
  std::list<Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  // set cruise speed
  FLAGS_default_cruise_speed = 10.0;

  // obstacle1
  perception_obstacle.mutable_position()->set_x(41000.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  perception_obstacle.mutable_velocity()->set_x(10.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  obstacle_buf.emplace_back("1", perception_obstacle);
  obstacles.emplace_back(&obstacle_buf.back());

  // obstacle2
  perception_obstacle.mutable_position()->set_x(25000.0);
  perception_obstacle.mutable_position()->set_y(0.0);
  perception_obstacle.mutable_velocity()->set_x(0.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  obstacle_buf.emplace_back("2", perception_obstacle);
  obstacles.emplace_back(&obstacle_buf.back());

  // obstacle3
  perception_obstacle.mutable_position()->set_x(10000.0);
  perception_obstacle.mutable_position()->set_y(-1.0);
  perception_obstacle.mutable_velocity()->set_x(-12.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  obstacle_buf.emplace_back("3", perception_obstacle);
  obstacles.emplace_back(&obstacle_buf.back());

  SpeedData speed_data;
  NaviSpeedDecider speed_decider;
  speed_decider.UpdateAccelSettings(2.0, 2.0, 4.0, 5.0);

  EXPECT_EQ(Status::OK(),
      speed_decider.MakeSpeedDecision(vehicle_state, obstacles, &speed_data));
  EXPECT_EQ(3, speed_data.speed_vector().size());

  const auto& first_point = speed_data.speed_vector()[0];
  const auto& second_point = speed_data.speed_vector()[1];
  const auto& third_point = speed_data.speed_vector()[2];

  EXPECT_NEAR(0.0, first_point.s(), 0.01);
  EXPECT_NEAR(0.0, first_point.t(), 0.01);
  EXPECT_NEAR(0.0, first_point.v(), 0.01);
  EXPECT_NEAR(2.0, first_point.a(), 0.01);
  EXPECT_NEAR(25.0, second_point.s(), 0.01);
  EXPECT_NEAR(5.0, second_point.t(), 0.01);
  EXPECT_NEAR(10.0, second_point.v(), 0.01);
  EXPECT_NEAR(0.0, second_point.a(), 0.01);
  EXPECT_NEAR(999.0, third_point.s(), 0.01);
  EXPECT_NEAR(102.4, third_point.t(), 0.01);
  EXPECT_NEAR(10.0, third_point.v(), 0.01);
  EXPECT_NEAR(0.0, third_point.a(), 0.01);
}

TEST(NaviSpeedDeciderTest, CreateSpeedData2) {
  VehicleState vehicle_state;
  vehicle_state.set_linear_velocity(10.0);

  PerceptionObstacle perception_obstacle;
  std::list<Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  // set cruise speed
  FLAGS_default_cruise_speed = 10.0;

  SpeedData speed_data;
  NaviSpeedDecider speed_decider;
  speed_decider.UpdateAccelSettings(2.0, 2.0, 4.0, 5.0);

  EXPECT_EQ(Status::OK(),
      speed_decider.MakeSpeedDecision(vehicle_state, obstacles, &speed_data));
  EXPECT_EQ(2, speed_data.speed_vector().size());

  const auto& first_point = speed_data.speed_vector()[0];
  const auto& second_point = speed_data.speed_vector()[1];

  EXPECT_NEAR(0.0, first_point.s(), 0.01);
  EXPECT_NEAR(0.0, first_point.t(), 0.01);
  EXPECT_NEAR(10.0, first_point.v(), 0.01);
  EXPECT_NEAR(0.0, first_point.a(), 0.01);
  EXPECT_NEAR(999.0, second_point.s(), 0.01);
  EXPECT_NEAR(99.9, second_point.t(), 0.01);
  EXPECT_NEAR(10.0, second_point.v(), 0.01);
  EXPECT_NEAR(0.0, second_point.a(), 0.01);
}

TEST(NaviSpeedDeciderTest, CreateSpeedData3) {
  VehicleState vehicle_state;
  vehicle_state.set_linear_velocity(5.0);

  PerceptionObstacle perception_obstacle;
  std::list<Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  // set cruise speed
  FLAGS_default_cruise_speed = 10.0;

  SpeedData speed_data;
  NaviSpeedDecider speed_decider;
  speed_decider.UpdateAccelSettings(2.0, 2.0, 4.0, 5.0);

  EXPECT_EQ(Status::OK(),
      speed_decider.MakeSpeedDecision(vehicle_state, obstacles, &speed_data));
  EXPECT_EQ(3, speed_data.speed_vector().size());

  const auto& first_point = speed_data.speed_vector()[0];
  const auto& second_point = speed_data.speed_vector()[1];
  const auto& third_point = speed_data.speed_vector()[2];

  EXPECT_NEAR(0.0, first_point.s(), 0.01);
  EXPECT_NEAR(0.0, first_point.t(), 0.01);
  EXPECT_NEAR(5.0, first_point.v(), 0.01);
  EXPECT_NEAR(2.0, first_point.a(), 0.01);
  EXPECT_NEAR(18.75, second_point.s(), 0.01);
  EXPECT_NEAR(2.5, second_point.t(), 0.01);
  EXPECT_NEAR(10.0, second_point.v(), 0.01);
  EXPECT_NEAR(0.0, second_point.a(), 0.01);
  EXPECT_NEAR(999.0, third_point.s(), 0.01);
  EXPECT_NEAR(100.525, third_point.t(), 0.01);
  EXPECT_NEAR(10.0, third_point.v(), 0.01);
  EXPECT_NEAR(0.0, third_point.a(), 0.01);
}

TEST(NaviSpeedDeciderTest, CreateSpeedData4) {
  VehicleState vehicle_state;
  vehicle_state.set_linear_velocity(20.0);

  PerceptionObstacle perception_obstacle;
  std::list<Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  // set cruise speed
  FLAGS_default_cruise_speed = 10.0;

  SpeedData speed_data;
  NaviSpeedDecider speed_decider;
  speed_decider.UpdateAccelSettings(2.0, 2.0, 4.0, 5.0);

  EXPECT_EQ(Status::OK(),
      speed_decider.MakeSpeedDecision(vehicle_state, obstacles, &speed_data));
  EXPECT_EQ(3, speed_data.speed_vector().size());

  const auto& first_point = speed_data.speed_vector()[0];
  const auto& second_point = speed_data.speed_vector()[1];
  const auto& third_point = speed_data.speed_vector()[2];

  EXPECT_NEAR(0.0, first_point.s(), 0.01);
  EXPECT_NEAR(0.0, first_point.t(), 0.01);
  EXPECT_NEAR(20.0, first_point.v(), 0.01);
  EXPECT_NEAR(-2.0, first_point.a(), 0.01);
  EXPECT_NEAR(75.0, second_point.s(), 0.01);
  EXPECT_NEAR(5.0, second_point.t(), 0.01);
  EXPECT_NEAR(10.0, second_point.v(), 0.01);
  EXPECT_NEAR(0.0, second_point.a(), 0.01);
  EXPECT_NEAR(999.0, third_point.s(), 0.01);
  EXPECT_NEAR(97.4, third_point.t(), 0.01);
  EXPECT_NEAR(10.0, third_point.v(), 0.01);
  EXPECT_NEAR(0.0, third_point.a(), 0.01);
}

TEST(NaviSpeedDeciderTest, ErrorTest) {
}

}  // namespace planning
}  // namespace apollo
