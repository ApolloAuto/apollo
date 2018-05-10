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

#include "modules/prediction/container/obstacles/obstacle.h"

#include <iostream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

#include "modules/perception/proto/perception_obstacle.pb.h"

#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

class ObstacleTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    FLAGS_p_var = 0.1;
    FLAGS_q_var = 0.1;
    FLAGS_r_var = 0.001;
    FLAGS_enable_kf_tracking = true;
    FLAGS_min_prediction_length = 50.0;
    FLAGS_adjust_velocity_by_position_shift = false;

    int num_frame = 3;
    for (int i = 1; i <= num_frame; ++i) {
      const auto filename = common::util::StrCat(
          "modules/prediction/testdata/frame_sequence/frame_", i, ".pb.txt");
      perception::PerceptionObstacles perception_obstacles;
      common::util::GetProtoFromFile(filename, &perception_obstacles);
      container_.Insert(perception_obstacles);
    }
  }

 protected:
  ObstaclesContainer container_;
};

TEST_F(ObstacleTest, VehicleBasic) {
  Obstacle* obstacle_ptr = container_.GetObstacle(1);
  EXPECT_TRUE(obstacle_ptr != nullptr);
  EXPECT_EQ(obstacle_ptr->id(), 1);
  EXPECT_EQ(obstacle_ptr->type(), perception::PerceptionObstacle::VEHICLE);
  EXPECT_TRUE(obstacle_ptr->IsOnLane());
  EXPECT_EQ(obstacle_ptr->history_size(), 3);
  EXPECT_DOUBLE_EQ(obstacle_ptr->timestamp(), 0.2);
}

TEST_F(ObstacleTest, VehiclePosition) {
  Obstacle* obstacle_ptr = container_.GetObstacle(1);

  const Feature& start_feature = obstacle_ptr->feature(2);
  EXPECT_DOUBLE_EQ(start_feature.timestamp(), 0.0);
  EXPECT_DOUBLE_EQ(start_feature.position().x(), -458.941);
  EXPECT_DOUBLE_EQ(start_feature.position().y(), -159.240);
  EXPECT_NEAR(start_feature.t_position().x(), -458.941, 0.001);
  EXPECT_NEAR(start_feature.t_position().y(), -159.240, 0.001);

  Feature* mid_feature_ptr = obstacle_ptr->mutable_feature(1);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->timestamp(), 0.1);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->position().x(), -457.010);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->position().y(), -160.023);
  EXPECT_NEAR(mid_feature_ptr->t_position().x(), -457.010, 0.1);
  EXPECT_NEAR(mid_feature_ptr->t_position().y(), -160.023, 0.1);

  const Feature& latest_feature = obstacle_ptr->latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature.timestamp(), 0.2);
  EXPECT_DOUBLE_EQ(latest_feature.position().x(), -455.182);
  EXPECT_DOUBLE_EQ(latest_feature.position().y(), -160.608);
  EXPECT_NEAR(latest_feature.t_position().x(), -455.182, 0.1);
  EXPECT_NEAR(latest_feature.t_position().y(), -160.608, 0.1);
}

TEST_F(ObstacleTest, VehicleVelocity) {
  Obstacle* obstacle_ptr = container_.GetObstacle(1);

  const Feature& start_feature = obstacle_ptr->feature(2);
  EXPECT_DOUBLE_EQ(start_feature.timestamp(), 0.0);
  EXPECT_DOUBLE_EQ(start_feature.velocity().x(), 18.794);
  EXPECT_DOUBLE_EQ(start_feature.velocity().y(), -6.839);

  const Feature& mid_feature = obstacle_ptr->feature(1);
  EXPECT_DOUBLE_EQ(mid_feature.timestamp(), 0.1);
  EXPECT_DOUBLE_EQ(mid_feature.velocity().x(), 18.796567195950544);
  EXPECT_DOUBLE_EQ(mid_feature.velocity().y(), -6.8439304092771129);

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature_ptr->timestamp(), 0.2);
  EXPECT_DOUBLE_EQ(latest_feature_ptr->velocity().x(), 18.786524599512003);
  EXPECT_DOUBLE_EQ(latest_feature_ptr->velocity().y(), -6.8246071588835804);

  EXPECT_NEAR(latest_feature_ptr->speed(), 19.987715462282193, 0.001);
}

TEST_F(ObstacleTest, VehicleHeading) {
  Obstacle* obstacle_ptr = container_.GetObstacle(1);
  const Feature& latest_feature = obstacle_ptr->latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature.theta(), -0.352);
}

TEST_F(ObstacleTest, VehicleLaneGraph) {
  Obstacle* obstacle_ptr = container_.GetObstacle(1);
  const Feature& latest_feature = obstacle_ptr->latest_feature();
  const LaneGraph& lane_graph = latest_feature.lane().lane_graph();
  EXPECT_EQ(lane_graph.lane_sequence_size(), 2);

  EXPECT_EQ(lane_graph.lane_sequence(0).lane_segment_size(), 3);
  EXPECT_EQ(lane_graph.lane_sequence(0).lane_segment(0).lane_id(), "l164");
  EXPECT_EQ(lane_graph.lane_sequence(0).lane_segment(1).lane_id(), "l120");
  EXPECT_EQ(lane_graph.lane_sequence(0).lane_segment(2).lane_id(), "l151");

  EXPECT_EQ(lane_graph.lane_sequence(1).lane_segment_size(), 3);
  EXPECT_EQ(lane_graph.lane_sequence(1).lane_segment(0).lane_id(), "l164");
  EXPECT_EQ(lane_graph.lane_sequence(1).lane_segment(1).lane_id(), "l35");
  EXPECT_EQ(lane_graph.lane_sequence(1).lane_segment(2).lane_id(), "l153");
}

TEST_F(ObstacleTest, PedestrianBasic) {
  Obstacle* obstacle_ptr = container_.GetObstacle(101);
  EXPECT_TRUE(obstacle_ptr != nullptr);
  EXPECT_EQ(obstacle_ptr->id(), 101);
  EXPECT_EQ(obstacle_ptr->type(), perception::PerceptionObstacle::PEDESTRIAN);
  EXPECT_EQ(obstacle_ptr->history_size(), 3);
  EXPECT_DOUBLE_EQ(obstacle_ptr->timestamp(), 0.2);
}

TEST_F(ObstacleTest, PedestrianPosition) {
  Obstacle* obstacle_ptr = container_.GetObstacle(101);

  const Feature& start_feature = obstacle_ptr->feature(2);
  EXPECT_DOUBLE_EQ(start_feature.timestamp(), 0.0);
  EXPECT_DOUBLE_EQ(start_feature.position().x(), -438.879);
  EXPECT_DOUBLE_EQ(start_feature.position().y(), -161.931);
  EXPECT_NEAR(start_feature.t_position().x(), -438.879, 0.001);
  EXPECT_NEAR(start_feature.t_position().y(), -161.931, 0.001);

  Feature* mid_feature_ptr = obstacle_ptr->mutable_feature(1);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->timestamp(), 0.1);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->position().x(), -438.610);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->position().y(), -161.521);
  EXPECT_NEAR(mid_feature_ptr->t_position().x(), -438.610, 0.05);
  EXPECT_NEAR(mid_feature_ptr->t_position().y(), -161.521, 0.05);

  const Feature& latest_feature = obstacle_ptr->latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature.timestamp(), 0.2);
  EXPECT_DOUBLE_EQ(latest_feature.position().x(), -438.537);
  EXPECT_DOUBLE_EQ(latest_feature.position().y(), -160.991);
  EXPECT_NEAR(latest_feature.t_position().x(), -438.537, 0.05);
  EXPECT_NEAR(latest_feature.t_position().y(), -160.991, 0.05);
}

TEST_F(ObstacleTest, PedestrianVelocity) {
  Obstacle* obstacle_ptr = container_.GetObstacle(101);

  const Feature& start_feature = obstacle_ptr->feature(2);
  EXPECT_DOUBLE_EQ(start_feature.timestamp(), 0.0);
  EXPECT_DOUBLE_EQ(start_feature.velocity().x(), 1.710);
  EXPECT_DOUBLE_EQ(start_feature.velocity().y(), 4.699);

  const Feature& mid_feature = obstacle_ptr->feature(1);
  EXPECT_DOUBLE_EQ(mid_feature.timestamp(), 0.1);
  EXPECT_DOUBLE_EQ(mid_feature.velocity().x(), 1.7148756822316562);
  EXPECT_DOUBLE_EQ(mid_feature.velocity().y(), 4.6960198636155503);

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature_ptr->timestamp(), 0.2);
  EXPECT_DOUBLE_EQ(latest_feature_ptr->velocity().x(), 1.6957282277561021);
  EXPECT_DOUBLE_EQ(latest_feature_ptr->velocity().y(), 4.7077623811976848);

  EXPECT_NEAR(latest_feature_ptr->speed(), 5.0038506033083108, 0.001);
}

TEST_F(ObstacleTest, PedestrianHeading) {
  Obstacle* obstacle_ptr = container_.GetObstacle(101);
  const Feature& latest_feature = obstacle_ptr->latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature.theta(), 1.220);
}

}  // namespace prediction
}  // namespace apollo
