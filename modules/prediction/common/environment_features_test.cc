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

#include "modules/prediction/common/environment_features.h"

#include "gtest/gtest.h"

namespace apollo {
namespace prediction {

class EnvironmentFeaturesTest : public ::testing::Test {
 public:
  void SetUp() override {}
};

TEST_F(EnvironmentFeaturesTest, EgoPosition) {
  EnvironmentFeatures environment_features;
  environment_features.set_ego_position(1.0, 2.0);
  const auto& ego_position = environment_features.get_ego_position();
  EXPECT_DOUBLE_EQ(ego_position.x(), 1.0);
  EXPECT_DOUBLE_EQ(ego_position.y(), 2.0);
}

TEST_F(EnvironmentFeaturesTest, EgoSpeed) {
  EnvironmentFeatures environment_features;
  environment_features.set_ego_speed(1.0);
  EXPECT_DOUBLE_EQ(environment_features.get_ego_speed(), 1.0);
}

TEST_F(EnvironmentFeaturesTest, EgoAcceleration) {
  EnvironmentFeatures environment_features;
  environment_features.set_ego_acceleration(1.0);
  EXPECT_DOUBLE_EQ(environment_features.get_ego_acceleration(), 1.0);
}

TEST_F(EnvironmentFeaturesTest, EgoHeading) {
  EnvironmentFeatures environment_features;
  environment_features.set_ego_heading(1.0);
  EXPECT_DOUBLE_EQ(environment_features.get_ego_heading(), 1.0);
}

TEST_F(EnvironmentFeaturesTest, EgoLane) {
  EnvironmentFeatures environment_features;
  EXPECT_FALSE(environment_features.has_ego_lane());
  environment_features.SetEgoLane("1-1", 1.0);
  EXPECT_TRUE(environment_features.has_ego_lane());
  auto ego_lane = environment_features.GetEgoLane();
  EXPECT_EQ(ego_lane.first, "1-1");
  EXPECT_DOUBLE_EQ(ego_lane.second, 1.0);
  environment_features.reset_ego_lane();
  EXPECT_FALSE(environment_features.has_ego_lane());
}

TEST_F(EnvironmentFeaturesTest, LeftLane) {
  EnvironmentFeatures environment_features;
  EXPECT_FALSE(environment_features.has_left_neighbor_lane());
  environment_features.SetLeftNeighborLane("1-1", 1.0);
  EXPECT_TRUE(environment_features.has_left_neighbor_lane());
  auto left_lane = environment_features.GetLeftNeighborLane();
  EXPECT_EQ(left_lane.first, "1-1");
  EXPECT_DOUBLE_EQ(left_lane.second, 1.0);
  environment_features.reset_left_neighbor_lane();
  EXPECT_FALSE(environment_features.has_left_neighbor_lane());
}

TEST_F(EnvironmentFeaturesTest, RightLane) {
  EnvironmentFeatures environment_features;
  EXPECT_FALSE(environment_features.has_right_neighbor_lane());
  environment_features.SetRightNeighborLane("1-1", 1.0);
  EXPECT_TRUE(environment_features.has_right_neighbor_lane());
  auto right_lane = environment_features.GetRightNeighborLane();
  EXPECT_EQ(right_lane.first, "1-1");
  EXPECT_DOUBLE_EQ(right_lane.second, 1.0);
  environment_features.reset_right_neighbor_lane();
  EXPECT_FALSE(environment_features.has_right_neighbor_lane());
}

TEST_F(EnvironmentFeaturesTest, FrontJunction) {
  EnvironmentFeatures environment_features;
  EXPECT_FALSE(environment_features.has_front_junction());
  environment_features.SetFrontJunction("2-1", 1.0);
  EXPECT_TRUE(environment_features.has_front_junction());
  auto junction = environment_features.GetFrontJunction();
  EXPECT_EQ(junction.first, "2-1");
  EXPECT_DOUBLE_EQ(junction.second, 1.0);
  environment_features.reset_front_junction();
  EXPECT_FALSE(environment_features.has_front_junction());
}

TEST_F(EnvironmentFeaturesTest, Obstacles) {
  EnvironmentFeatures environment_features;
  environment_features.AddObstacleId(1);
  environment_features.AddObstacleId(2);
  EXPECT_EQ(environment_features.get_obstacle_ids().size(), 2);
}

}  // namespace prediction
}  // namespace apollo
