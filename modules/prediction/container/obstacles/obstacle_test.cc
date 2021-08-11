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

#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
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
    FLAGS_enable_kf_tracking = false;
    FLAGS_min_prediction_trajectory_spatial_length = 50.0;
    FLAGS_adjust_velocity_by_position_shift = false;
    FLAGS_adjust_vehicle_heading_by_lane = false;

    int num_frame = 3;
    for (int i = 1; i <= num_frame; ++i) {
      const auto filename = absl::StrCat(
          "modules/prediction/testdata/frame_sequence/frame_", i, ".pb.txt");
      perception::PerceptionObstacles perception_obstacles;
      cyber::common::GetProtoFromFile(filename, &perception_obstacles);
      container_.Insert(perception_obstacles);
      container_.BuildLaneGraph();
    }
  }

 protected:
  ObstaclesContainer container_;
};

TEST_F(ObstacleTest, VehicleBasic) {
  Obstacle* obstacle_ptr = container_.GetObstacle(1);
  EXPECT_NE(obstacle_ptr, nullptr);
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

  Feature* mid_feature_ptr = obstacle_ptr->mutable_feature(1);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->timestamp(), 0.1);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->position().x(), -457.010);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->position().y(), -160.023);

  const Feature& latest_feature = obstacle_ptr->latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature.timestamp(), 0.2);
  EXPECT_DOUBLE_EQ(latest_feature.position().x(), -455.182);
  EXPECT_DOUBLE_EQ(latest_feature.position().y(), -160.608);
}

TEST_F(ObstacleTest, VehicleVelocity) {
  Obstacle* obstacle_ptr = container_.GetObstacle(1);

  const Feature& start_feature = obstacle_ptr->feature(2);
  EXPECT_DOUBLE_EQ(start_feature.timestamp(), 0.0);
  EXPECT_DOUBLE_EQ(start_feature.velocity().x(), 18.794);
  EXPECT_DOUBLE_EQ(start_feature.velocity().y(), -6.839);

  const Feature& mid_feature = obstacle_ptr->feature(1);
  EXPECT_DOUBLE_EQ(mid_feature.timestamp(), 0.1);
  EXPECT_DOUBLE_EQ(mid_feature.velocity().x(), 17.994);
  EXPECT_DOUBLE_EQ(mid_feature.velocity().y(), -6.8390000000000004);

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature_ptr->timestamp(), 0.2);
  EXPECT_DOUBLE_EQ(latest_feature_ptr->velocity().x(), 17.994);
  EXPECT_DOUBLE_EQ(latest_feature_ptr->velocity().y(), -6.8390000000000004);

  EXPECT_NEAR(latest_feature_ptr->speed(), 19.249830051197854, 0.001);
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
  EXPECT_NE(obstacle_ptr, nullptr);
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

  Feature* mid_feature_ptr = obstacle_ptr->mutable_feature(1);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->timestamp(), 0.1);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->position().x(), -438.610);
  EXPECT_DOUBLE_EQ(mid_feature_ptr->position().y(), -161.521);

  const Feature& latest_feature = obstacle_ptr->latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature.timestamp(), 0.2);
  EXPECT_DOUBLE_EQ(latest_feature.position().x(), -438.537);
  EXPECT_DOUBLE_EQ(latest_feature.position().y(), -160.991);
}

TEST_F(ObstacleTest, PedestrianVelocity) {
  Obstacle* obstacle_ptr = container_.GetObstacle(101);

  const Feature& start_feature = obstacle_ptr->feature(2);
  EXPECT_DOUBLE_EQ(start_feature.timestamp(), 0.0);
  EXPECT_DOUBLE_EQ(start_feature.velocity().x(), 1.710);
  EXPECT_DOUBLE_EQ(start_feature.velocity().y(), 4.699);

  const Feature& mid_feature = obstacle_ptr->feature(1);
  EXPECT_DOUBLE_EQ(mid_feature.timestamp(), 0.1);
  EXPECT_DOUBLE_EQ(mid_feature.velocity().x(), 1.710);
  EXPECT_DOUBLE_EQ(mid_feature.velocity().y(), 4.699);

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature_ptr->timestamp(), 0.2);
  EXPECT_DOUBLE_EQ(latest_feature_ptr->velocity().x(), 1.710);
  EXPECT_DOUBLE_EQ(latest_feature_ptr->velocity().y(), 4.699);

  EXPECT_NEAR(latest_feature_ptr->speed(), 5.0004700779026763, 0.001);
}

TEST_F(ObstacleTest, PedestrianHeading) {
  Obstacle* obstacle_ptr = container_.GetObstacle(101);
  const Feature& latest_feature = obstacle_ptr->latest_feature();
  EXPECT_DOUBLE_EQ(latest_feature.theta(), 1.220);
}

TEST_F(ObstacleTest, Priority) {
  Obstacle* obstacle_ptr = container_.GetObstacle(101);
  EXPECT_FALSE(obstacle_ptr->ToIgnore());
}

}  // namespace prediction
}  // namespace apollo
