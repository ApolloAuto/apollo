/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/predictor/interaction/interaction_predictor.h"

#include "cyber/common/file.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"

namespace apollo {
namespace prediction {

class InteractionPredictorTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    const std::string file =
        "modules/prediction/testdata/single_perception_vehicle_onlane.pb.txt";
    cyber::common::GetProtoFromFile(file, &perception_obstacles_);
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
};

TEST_F(InteractionPredictorTest, OnLaneCase) {
  EXPECT_DOUBLE_EQ(perception_obstacles_.header().timestamp_sec(),
                   1501183430.161906);
  apollo::perception::PerceptionObstacle perception_obstacle =
      perception_obstacles_.perception_obstacle(0);
  EXPECT_EQ(perception_obstacle.id(), 1);
  MLPEvaluator mlp_evaluator;
  ObstaclesContainer container;
  ADCTrajectoryContainer adc_trajectory_container;
  container.Insert(perception_obstacles_);
  container.BuildLaneGraph();
  Obstacle* obstacle_ptr = container.GetObstacle(1);
  EXPECT_NE(obstacle_ptr, nullptr);
  mlp_evaluator.Evaluate(obstacle_ptr, &container);
  InteractionPredictor predictor;
  predictor.Predict(&adc_trajectory_container, obstacle_ptr, &container);
  EXPECT_EQ(predictor.NumOfTrajectories(*obstacle_ptr), 1);
}

}  // namespace prediction
}  // namespace apollo
