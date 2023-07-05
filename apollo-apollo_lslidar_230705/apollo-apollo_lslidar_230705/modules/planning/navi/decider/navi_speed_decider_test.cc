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

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "modules/common/util/point_factory.h"
#include "modules/planning/common/planning_gflags.h"

using apollo::common::PathPoint;
using apollo::common::Status;
using apollo::common::util::PointFactory;
using apollo::perception::PerceptionObstacle;
using apollo::prediction::ObstaclePriority;

namespace apollo {
namespace planning {

static PathPoint GenPathPoint(double s, double kappa = 0.0) {
  return PointFactory::ToPathPoint(s, 0.0, 0.0, s, 0.0, kappa);
}

TEST(NaviSpeedDeciderTest, CreateSpeedData) {
  NaviSpeedDecider speed_decider;
  speed_decider.preferred_speed_ = 10.0;
  speed_decider.max_speed_ = 20.0;
  speed_decider.preferred_accel_ = 2.0;
  speed_decider.preferred_decel_ = 2.0;
  speed_decider.preferred_jerk_ = 2.0;
  speed_decider.max_accel_ = 5.0;
  speed_decider.max_decel_ = 5.0;
  speed_decider.obstacle_buffer_ = 1.0;
  speed_decider.safe_distance_base_ = 2.0;
  speed_decider.safe_distance_ratio_ = 1.0;
  speed_decider.following_accel_ratio_ = 0.5;
  speed_decider.soft_centric_accel_limit_ = 1.2;
  speed_decider.hard_centric_accel_limit_ = 1.5;
  speed_decider.hard_speed_limit_ = 100.0;
  speed_decider.hard_accel_limit_ = 10.0;
  speed_decider.enable_safe_path_ = true;
  speed_decider.enable_planning_start_point_ = true;
  speed_decider.kappa_preview_ = 0.0;
  speed_decider.kappa_threshold_ = 0.0;

  PerceptionObstacle perception_obstacle;
  std::map<std::string, Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  std::vector<PathPoint> path_points;
  path_points.emplace_back(GenPathPoint(0.0));
  path_points.emplace_back(GenPathPoint(100.0));

  SpeedData speed_data;
  EXPECT_EQ(Status::OK(), speed_decider.MakeSpeedDecision(
                              0.0, 0.0, 0.0, path_points, obstacles,
                              [&](const std::string& id) mutable {
                                return &obstacle_buf[id];
                              },
                              &speed_data));

  for (auto& p : speed_data) {
    if (p.s() > 2.0 && p.s() < 24.0) {
      EXPECT_NEAR(2.0, p.a(), 0.1);
    }
    if (p.s() > 26.0 && p.s() < 60.0) {
      EXPECT_NEAR(10.0, p.v(), 0.1);
    }
  }
}

TEST(NaviSpeedDeciderTest, CreateSpeedDataForStaticObstacle) {
  NaviSpeedDecider speed_decider;
  speed_decider.preferred_speed_ = 10.0;
  speed_decider.max_speed_ = 20.0;
  speed_decider.preferred_accel_ = 1.0;
  speed_decider.preferred_decel_ = 1.0;
  speed_decider.preferred_jerk_ = 2.0;
  speed_decider.max_accel_ = 5.0;
  speed_decider.max_decel_ = 5.0;
  speed_decider.obstacle_buffer_ = 1.0;
  speed_decider.safe_distance_base_ = 2.0;
  speed_decider.safe_distance_ratio_ = 1.0;
  speed_decider.following_accel_ratio_ = 0.5;
  speed_decider.soft_centric_accel_limit_ = 1.2;
  speed_decider.hard_centric_accel_limit_ = 1.5;
  speed_decider.hard_speed_limit_ = 100.0;
  speed_decider.hard_accel_limit_ = 10.0;
  speed_decider.enable_safe_path_ = false;
  speed_decider.enable_planning_start_point_ = true;
  speed_decider.kappa_preview_ = 0.0;
  speed_decider.kappa_threshold_ = 0.0;

  PerceptionObstacle perception_obstacle;
  std::map<std::string, Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  std::vector<PathPoint> path_points;
  path_points.emplace_back(GenPathPoint(0.0));
  path_points.emplace_back(GenPathPoint(200.0));

  // obstacle1
  perception_obstacle.mutable_position()->set_x(30.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  perception_obstacle.mutable_velocity()->set_x(0.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  std::string id = "1";
  obstacle_buf.emplace(
      id, Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, false));
  obstacles.emplace_back(&obstacle_buf[id]);

  SpeedData speed_data;
  EXPECT_EQ(Status::OK(), speed_decider.MakeSpeedDecision(
                              0.0, 0.0, 0.0, path_points, obstacles,
                              [&](const std::string& id) mutable {
                                return &obstacle_buf[id];
                              },
                              &speed_data));
  for (auto& p : speed_data) {
    if (p.s() > 16.7) {
      EXPECT_NEAR(0.0, p.v(), 1.0);
    }
  }
}

TEST(NaviSpeedDeciderTest, CreateSpeedDataForObstacles) {
  NaviSpeedDecider speed_decider;
  speed_decider.preferred_speed_ = 10.0;
  speed_decider.max_speed_ = 20.0;
  speed_decider.preferred_accel_ = 1.0;
  speed_decider.preferred_decel_ = 1.0;
  speed_decider.preferred_jerk_ = 2.0;
  speed_decider.max_accel_ = 5.0;
  speed_decider.max_decel_ = 5.0;
  speed_decider.obstacle_buffer_ = 1.0;
  speed_decider.safe_distance_base_ = 2.0;
  speed_decider.safe_distance_ratio_ = 1.0;
  speed_decider.following_accel_ratio_ = 0.5;
  speed_decider.soft_centric_accel_limit_ = 1.2;
  speed_decider.hard_centric_accel_limit_ = 1.5;
  speed_decider.hard_speed_limit_ = 100.0;
  speed_decider.hard_accel_limit_ = 10.0;
  speed_decider.enable_safe_path_ = true;
  speed_decider.enable_planning_start_point_ = true;
  speed_decider.kappa_preview_ = 0.0;
  speed_decider.kappa_threshold_ = 0.0;

  PerceptionObstacle perception_obstacle;
  std::map<std::string, Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  std::vector<PathPoint> path_points;
  path_points.emplace_back(GenPathPoint(0.0));
  path_points.emplace_back(GenPathPoint(100.0));

  // obstacle1
  perception_obstacle.mutable_position()->set_x(50.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  perception_obstacle.mutable_velocity()->set_x(0.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  std::string id = "1";
  obstacle_buf.emplace(
      id, Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, false));
  obstacles.emplace_back(&obstacle_buf[id]);

  // obstacle2
  perception_obstacle.mutable_position()->set_x(20.0);
  perception_obstacle.mutable_position()->set_y(1.0);
  perception_obstacle.mutable_velocity()->set_x(5.0);
  perception_obstacle.mutable_velocity()->set_y(0.0);
  perception_obstacle.set_length(3.0);
  perception_obstacle.set_width(3.0);
  id = "2";
  obstacle_buf.emplace(
      id, Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL, false));
  obstacles.emplace_back(&obstacle_buf[id]);

  SpeedData speed_data;
  EXPECT_EQ(Status::OK(), speed_decider.MakeSpeedDecision(
                              10.0, 0.0, 0.0, path_points, obstacles,
                              [&](const std::string& id) mutable {
                                return &obstacle_buf[id];
                              },
                              &speed_data));
  for (auto& p : speed_data) {
    if (p.s() > 15.0 && p.s() < 26.0) {
      EXPECT_NEAR(5.0, p.v(), 0.5);
    }
    if (p.s() > 37.0) {
      EXPECT_NEAR(0.0, p.v(), 1.0);
    }
  }
}

TEST(NaviSpeedDeciderTest, CreateSpeedDataForCurve) {
  NaviSpeedDecider speed_decider;
  speed_decider.preferred_speed_ = 12.0;
  speed_decider.max_speed_ = 30.0;
  speed_decider.preferred_accel_ = 2.0;
  speed_decider.preferred_decel_ = 1.5;
  speed_decider.preferred_jerk_ = 2.0;
  speed_decider.max_accel_ = 5.0;
  speed_decider.max_decel_ = 5.0;
  speed_decider.obstacle_buffer_ = 1.0;
  speed_decider.safe_distance_base_ = 2.0;
  speed_decider.safe_distance_ratio_ = 1.0;
  speed_decider.following_accel_ratio_ = 0.5;
  speed_decider.soft_centric_accel_limit_ = 1.0;
  speed_decider.hard_centric_accel_limit_ = 1.5;
  speed_decider.hard_speed_limit_ = 100.0;
  speed_decider.hard_accel_limit_ = 10.0;
  speed_decider.enable_safe_path_ = false;
  speed_decider.enable_planning_start_point_ = true;
  speed_decider.kappa_preview_ = 0.0;
  speed_decider.kappa_threshold_ = 0.0;

  PerceptionObstacle perception_obstacle;
  std::map<std::string, Obstacle> obstacle_buf;
  std::vector<const Obstacle*> obstacles;

  std::vector<PathPoint> path_points;
  double s = 0.0;
  path_points.emplace_back(GenPathPoint(s, 0.0));
  s += 50.0;
  path_points.emplace_back(GenPathPoint(s, 0.0));
  for (size_t i = 1; i <= 5; i++) {
    s += 1.0;
    path_points.emplace_back(GenPathPoint(s, 0.03 * static_cast<double>(i)));
  }
  for (size_t i = 1; i <= 5; i++) {
    s += 1.0;
    path_points.emplace_back(GenPathPoint(s, 0.15));
  }
  for (size_t i = 1; i <= 5; i++) {
    s += 1.0;
    path_points.emplace_back(
        GenPathPoint(s, 0.03 * static_cast<double>(5 - i)));
  }
  s += 1.0;
  path_points.emplace_back(GenPathPoint(s, 0.0));
  s += 10.0;
  path_points.emplace_back(GenPathPoint(s, 0.0));
  for (size_t i = 1; i <= 10; i++) {
    s += 1.0;
    path_points.emplace_back(GenPathPoint(s, -0.07));
  }
  for (size_t i = 1; i <= 10; i++) {
    s += 1.0;
    path_points.emplace_back(GenPathPoint(s, 0.07));
  }
  s += 1.0;
  path_points.emplace_back(GenPathPoint(s, 0.0));
  s += 20.0;
  path_points.emplace_back(GenPathPoint(s, 0.0));

  SpeedData speed_data;
  EXPECT_EQ(Status::OK(), speed_decider.MakeSpeedDecision(
                              12.1, 0.0, 0.0, path_points, obstacles,
                              [&](const std::string& id) mutable {
                                return &obstacle_buf[id];
                              },
                              &speed_data));
  for (auto& p : speed_data) {
    if (p.s() > 56.0 && p.s() < 59.0) {
      EXPECT_NEAR(2.6, p.v(), 0.1);
    }
    if (p.s() > 88.0 && p.s() < 95.0) {
      EXPECT_NEAR(3.7, p.v(), 0.1);
    }
  }
}

TEST(NaviSpeedDeciderTest, ErrorTest) {}

}  // namespace planning
}  // namespace apollo
