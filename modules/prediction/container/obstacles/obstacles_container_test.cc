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

#include "cyber/common/file.h"
#include "modules/prediction/common/kml_map_based_test.h"

namespace apollo {
namespace prediction {

class ObstaclesContainerTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    const std::string file =
        "modules/prediction/testdata/perception_vehicles_pedestrians.pb.txt";
    perception::PerceptionObstacles perception_obstacles;
    cyber::common::GetProtoFromFile(file, &perception_obstacles);
    container_.Insert(perception_obstacles);
  }

 protected:
  ObstaclesContainer container_;
};

TEST_F(ObstaclesContainerTest, Vehicles) {
  Obstacle* obstacle_ptr0 = container_.GetObstacle(0);
  EXPECT_NE(nullptr, obstacle_ptr0);
  EXPECT_EQ(obstacle_ptr0->id(), 0);
  EXPECT_EQ(obstacle_ptr0->type(), perception::PerceptionObstacle::VEHICLE);
  Obstacle* obstacle_ptr1 = container_.GetObstacle(1);
  EXPECT_NE(nullptr, obstacle_ptr1);
  EXPECT_EQ(obstacle_ptr1->id(), 1);
  EXPECT_EQ(obstacle_ptr1->type(), perception::PerceptionObstacle::VEHICLE);
  Obstacle* obstacle_ptr2 = container_.GetObstacle(2);
  EXPECT_NE(nullptr, obstacle_ptr2);
  EXPECT_EQ(obstacle_ptr2->id(), 2);
  EXPECT_EQ(obstacle_ptr2->type(), perception::PerceptionObstacle::VEHICLE);
  Obstacle* obstacle_ptr3 = container_.GetObstacle(3);
  EXPECT_NE(nullptr, obstacle_ptr3);
  EXPECT_EQ(obstacle_ptr3->id(), 3);
  EXPECT_EQ(obstacle_ptr3->type(), perception::PerceptionObstacle::VEHICLE);
  Obstacle* obstacle_ptr4 = container_.GetObstacle(4);
  EXPECT_EQ(nullptr, obstacle_ptr4);

  EXPECT_EQ(container_.curr_frame_movable_obstacle_ids().size(), 6);
}

TEST_F(ObstaclesContainerTest, Pedestrian) {
  Obstacle* obstacle_ptr101 = container_.GetObstacle(101);
  EXPECT_NE(nullptr, obstacle_ptr101);
  EXPECT_EQ(obstacle_ptr101->id(), 101);
  EXPECT_EQ(obstacle_ptr101->type(),
            perception::PerceptionObstacle::PEDESTRIAN);
  Obstacle* obstacle_ptr102 = container_.GetObstacle(102);
  EXPECT_NE(nullptr, obstacle_ptr102);
  EXPECT_EQ(obstacle_ptr102->id(), 102);
  EXPECT_EQ(obstacle_ptr102->type(),
            perception::PerceptionObstacle::PEDESTRIAN);
  Obstacle* obstacle_ptr103 = container_.GetObstacle(103);
  EXPECT_EQ(nullptr, obstacle_ptr103);
}

TEST_F(ObstaclesContainerTest, ClearAll) {
  container_.Clear();
  EXPECT_EQ(nullptr, container_.GetObstacle(0));
  EXPECT_EQ(nullptr, container_.GetObstacle(1));
  EXPECT_EQ(nullptr, container_.GetObstacle(2));
  EXPECT_EQ(nullptr, container_.GetObstacle(3));
  EXPECT_EQ(nullptr, container_.GetObstacle(101));
  EXPECT_EQ(nullptr, container_.GetObstacle(102));
}

}  // namespace prediction
}  // namespace apollo
