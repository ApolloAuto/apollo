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

#include "modules/prediction/predictor/free_move/free_move_predictor.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/util/file.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace apollo {
namespace prediction {

class FreeMovePredictorTest : public KMLMapBasedTest {
 public:
  FreeMovePredictorTest() {
    CHECK(apollo::common::util::GetProtoFromFile(
        "modules/prediction/testdata/single_perception_vehicle_offlane.pb.txt",
        &perception_obstacles_));
    FLAGS_p_var = 0.1;
    FLAGS_q_var = 0.01;
    FLAGS_r_var = 0.25;
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
};

TEST_F(FreeMovePredictorTest, General) {
  EXPECT_DOUBLE_EQ(perception_obstacles_.header().timestamp_sec(),
                   1501183430.161906);
  apollo::perception::PerceptionObstacle perception_obstacle =
      perception_obstacles_.perception_obstacle(0);
  EXPECT_EQ(perception_obstacle.id(), 15);
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  Obstacle* obstacle_ptr = container.GetObstacle(15);
  EXPECT_TRUE(obstacle_ptr != nullptr);
  FreeMovePredictor predictor;
  predictor.Predict(obstacle_ptr);
  const std::vector<Trajectory>& trajectories = predictor.trajectories();
  EXPECT_EQ(trajectories.size(), 1);
  EXPECT_EQ(trajectories[0].trajectory_point_size(), 50);
  EXPECT_NEAR(trajectories[0].trajectory_point(9).path_point().x(), -432.459,
              0.001);
  EXPECT_NEAR(trajectories[0].trajectory_point(9).path_point().y(), -156.451,
              0.001);
}

TEST_F(FreeMovePredictorTest, Pedestrian) {
  perception_obstacles_.mutable_perception_obstacle(0)->set_type(
      ::apollo::perception::PerceptionObstacle::PEDESTRIAN);
  apollo::perception::PerceptionObstacle perception_obstacle =
      perception_obstacles_.perception_obstacle(0);
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  Obstacle* obstacle_ptr = container.GetObstacle(15);
  FreeMovePredictor predictor;
  predictor.Predict(obstacle_ptr);
  const std::vector<Trajectory>& trajectories = predictor.trajectories();
  EXPECT_EQ(trajectories.size(), 1);
  EXPECT_EQ(trajectories[0].trajectory_point_size(), 100);
}

}  // namespace prediction
}  // namespace apollo
