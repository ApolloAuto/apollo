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

#include "modules/prediction/evaluator/vehicle/junction_mlp_evaluator.h"

#include "cyber/common/file.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

class JunctionMLPEvaluatorTest : public KMLMapBasedTest {
 public:
  void SetUp() override {
    const std::string file =
        "modules/prediction/testdata/"
        "single_perception_vehicle_injunction.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(file, &perception_obstacles_));
    FLAGS_enable_all_junction = true;
    JunctionAnalyzer::Init("j2");
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
};

TEST_F(JunctionMLPEvaluatorTest, InJunctionCase) {
  EXPECT_DOUBLE_EQ(perception_obstacles_.header().timestamp_sec(),
                   1501183430.161906);
  apollo::perception::PerceptionObstacle perception_obstacle =
      perception_obstacles_.perception_obstacle(0);
  EXPECT_EQ(perception_obstacle.id(), 1);
  JunctionMLPEvaluator junction_mlp_evaluator;
  ObstaclesContainer container;
  container.Insert(perception_obstacles_);
  container.BuildJunctionFeature();
  Obstacle* obstacle_ptr = container.GetObstacle(1);
  EXPECT_NE(obstacle_ptr, nullptr);
  junction_mlp_evaluator.Evaluate(obstacle_ptr, &container);
  const JunctionFeature& junction_feature =
      obstacle_ptr->latest_feature().junction_feature();
  EXPECT_EQ(junction_feature.junction_id(), "j2");
  EXPECT_GT(junction_feature.junction_exit_size(), 0);
  for (const auto& junction_exit : junction_feature.junction_exit()) {
    EXPECT_TRUE(junction_exit.has_exit_lane_id());
  }
  EXPECT_EQ(junction_feature.junction_mlp_probability_size(), 12);
  junction_mlp_evaluator.Clear();
}

}  // namespace prediction
}  // namespace apollo
