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

#include "modules/prediction/predictor/regional/regional_predictor.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

#include "modules/common/util/file.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

class RegionalPredictorTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    std::string file =
        "modules/prediction/testdata/multiple_perception_pedestrians.pb.txt";
    apollo::common::util::GetProtoFromFile(file, &perception_obstacles_);
    FLAGS_p_var = 0.1;
    FLAGS_q_var = 0.01;
    FLAGS_r_var = 0.25;
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
};

TEST_F(RegionalPredictorTest, MovingPedestrian) {
  EXPECT_DOUBLE_EQ(perception_obstacles_.header().timestamp_sec(),
                   1501183430.161906);
  apollo::perception::PerceptionObstacle perception_obstacle =
      perception_obstacles_.perception_obstacle(0);
  EXPECT_EQ(perception_obstacle.id(), 101);
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  Obstacle* obstacle_ptr = container.GetObstacle(101);
  EXPECT_TRUE(obstacle_ptr != nullptr);
  RegionalPredictor predictor;
  predictor.Predict(obstacle_ptr);
  const std::vector<Trajectory>& trajectories = predictor.trajectories();
  EXPECT_EQ(trajectories.size(), 2);
  EXPECT_NEAR(trajectories[0].trajectory_point(9).path_point().x(), -438.159,
              0.001);
  EXPECT_NEAR(trajectories[0].trajectory_point(9).path_point().y(), -157.404,
              0.001);
  EXPECT_NEAR(trajectories[0].trajectory_point(19).path_point().x(), -436.642,
              0.001);
  EXPECT_NEAR(trajectories[0].trajectory_point(19).path_point().y(), -152.635,
              0.001);
  EXPECT_NEAR(trajectories[1].trajectory_point(9).path_point().x(), -436.521,
              0.001);
  EXPECT_NEAR(trajectories[1].trajectory_point(9).path_point().y(), -158.000,
              0.001);
  EXPECT_NEAR(trajectories[1].trajectory_point(19).path_point().x(), -434.618,
              0.001);
  EXPECT_NEAR(trajectories[1].trajectory_point(19).path_point().y(), -153.371,
              0.001);
}

TEST_F(RegionalPredictorTest, StationaryPedestrian) {
  EXPECT_DOUBLE_EQ(perception_obstacles_.header().timestamp_sec(),
                   1501183430.161906);
  apollo::perception::PerceptionObstacle perception_obstacle =
      perception_obstacles_.perception_obstacle(1);
  EXPECT_EQ(perception_obstacle.id(), 102);
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  Obstacle* obstacle_ptr = container.GetObstacle(102);
  EXPECT_TRUE(obstacle_ptr != nullptr);
  RegionalPredictor predictor;
  predictor.Predict(obstacle_ptr);
  const std::vector<Trajectory>& trajectories = predictor.trajectories();
  EXPECT_EQ(trajectories.size(), 1);
}

}  // namespace prediction
}  // namespace apollo
