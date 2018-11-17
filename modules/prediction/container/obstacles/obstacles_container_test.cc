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

#include "modules/prediction/container/obstacles/obstacles_container.h"

#include <string>

#include "gtest/gtest.h"

#include "modules/common/util/file.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/obstacles/obstacle.h"

namespace apollo {
namespace prediction {

class ObstaclesContainerTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    std::string file =
        "modules/prediction/testdata/perception_vehicles_pedestrians.pb.txt";
    perception::PerceptionObstacles perception_obstacles;
    common::util::GetProtoFromFile(file, &perception_obstacles);
    container_.Insert(perception_obstacles);
  }

 protected:
  ObstaclesContainer container_;
};

TEST_F(ObstaclesContainerTest, Vehicles) {
  Obstacle* obstacle_ptr0 = container_.GetObstacle(0);
  EXPECT_TRUE(obstacle_ptr0 != nullptr);
  EXPECT_EQ(obstacle_ptr0->id(), 0);
  EXPECT_EQ(obstacle_ptr0->type(), perception::PerceptionObstacle::VEHICLE);
  Obstacle* obstacle_ptr1 = container_.GetObstacle(1);
  EXPECT_TRUE(obstacle_ptr1 != nullptr);
  EXPECT_EQ(obstacle_ptr1->id(), 1);
  EXPECT_EQ(obstacle_ptr1->type(), perception::PerceptionObstacle::VEHICLE);
  Obstacle* obstacle_ptr2 = container_.GetObstacle(2);
  EXPECT_TRUE(obstacle_ptr2 != nullptr);
  EXPECT_EQ(obstacle_ptr2->id(), 2);
  EXPECT_EQ(obstacle_ptr2->type(), perception::PerceptionObstacle::VEHICLE);
  Obstacle* obstacle_ptr3 = container_.GetObstacle(3);
  EXPECT_TRUE(obstacle_ptr3 != nullptr);
  EXPECT_EQ(obstacle_ptr3->id(), 3);
  EXPECT_EQ(obstacle_ptr3->type(), perception::PerceptionObstacle::VEHICLE);
  Obstacle* obstacle_ptr4 = container_.GetObstacle(4);
  EXPECT_TRUE(obstacle_ptr4 == nullptr);
}

TEST_F(ObstaclesContainerTest, Pedestrian) {
  Obstacle* obstacle_ptr101 = container_.GetObstacle(101);
  EXPECT_TRUE(obstacle_ptr101 != nullptr);
  EXPECT_EQ(obstacle_ptr101->id(), 101);
  EXPECT_EQ(obstacle_ptr101->type(),
            perception::PerceptionObstacle::PEDESTRIAN);
  Obstacle* obstacle_ptr102 = container_.GetObstacle(102);
  EXPECT_TRUE(obstacle_ptr102 != nullptr);
  EXPECT_EQ(obstacle_ptr102->id(), 102);
  EXPECT_EQ(obstacle_ptr102->type(),
            perception::PerceptionObstacle::PEDESTRIAN);
  Obstacle* obstacle_ptr103 = container_.GetObstacle(103);
  EXPECT_TRUE(obstacle_ptr103 == nullptr);
}

TEST_F(ObstaclesContainerTest, ClearAll) {
  container_.Clear();
  EXPECT_TRUE(container_.GetObstacle(0) == nullptr);
  EXPECT_TRUE(container_.GetObstacle(1) == nullptr);
  EXPECT_TRUE(container_.GetObstacle(2) == nullptr);
  EXPECT_TRUE(container_.GetObstacle(3) == nullptr);
  EXPECT_TRUE(container_.GetObstacle(101) == nullptr);
  EXPECT_TRUE(container_.GetObstacle(102) == nullptr);
}

}  // namespace prediction
}  // namespace apollo
