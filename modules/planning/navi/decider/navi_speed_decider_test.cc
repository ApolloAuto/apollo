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

TEST(NaviSpeedDeciderTest, CreateSpeedData) {
  VehicleState vehicle_state;
  vehicle_state.set_linear_velocity(30.0);
  vehicle_state.set_linear_acceleration(1.0);

  PerceptionObstacle perception_obstacle;
  std::list<Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  // set cruise speed
  FLAGS_default_cruise_speed = 60.0;

  // obstacle1
  perception_obstacle.mutable_position()->set_x(41.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  perception_obstacle.mutable_velocity()->set_x(10.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  obstacle_buf.emplace_back("1", perception_obstacle);
  obstacles.emplace_back(&obstacle_buf.back());

  // obstacle2
  perception_obstacle.mutable_position()->set_x(25.0);
  perception_obstacle.mutable_position()->set_y(0.0);
  perception_obstacle.mutable_velocity()->set_x(0.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  obstacle_buf.emplace_back("2", perception_obstacle);
  obstacles.emplace_back(&obstacle_buf.back());

  // obstacle3
  perception_obstacle.mutable_position()->set_x(10.0);
  perception_obstacle.mutable_position()->set_y(-1.0);
  perception_obstacle.mutable_velocity()->set_x(-12.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  obstacle_buf.emplace_back("3", perception_obstacle);
  obstacles.emplace_back(&obstacle_buf.back());

  SpeedData speed_data;
  NaviSpeedDecider speed_decider;
  EXPECT_EQ(Status::OK(),
      speed_decider.MakeSpeedDecision(vehicle_state, obstacles, &speed_data));
  EXPECT_EQ(2, speed_data.speed_vector().size());
  EXPECT_DOUBLE_EQ(0.0, speed_data.speed_vector().front().s());
  EXPECT_DOUBLE_EQ(0.0, speed_data.speed_vector().front().t());
  EXPECT_DOUBLE_EQ(18.0, speed_data.speed_vector().front().v());
  EXPECT_DOUBLE_EQ(1.0, speed_data.speed_vector().front().a());
  EXPECT_NEAR(4.61, speed_data.speed_vector().back().s(), 0.01);
  EXPECT_NEAR(0.256, speed_data.speed_vector().back().t(), 0.001);
  EXPECT_DOUBLE_EQ(18.0, speed_data.speed_vector().back().v());
  EXPECT_DOUBLE_EQ(0.0, speed_data.speed_vector().back().a());
}

TEST(NaviSpeedDeciderTest, ErrorTest) {
}

}  // namespace planning
}  // namespace apollo
