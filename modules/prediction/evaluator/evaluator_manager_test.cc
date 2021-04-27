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

#include "modules/prediction/evaluator/evaluator_manager.h"

#include "cyber/common/file.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;

class EvaluatorManagerTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    const std::string file =
        "modules/prediction/testdata/single_perception_vehicle_onlane.pb.txt";
    ACHECK(cyber::common::GetProtoFromFile(file, &perception_obstacles_));
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
  common::adapter::AdapterManagerConfig adapter_conf_;
  PredictionConf prediction_conf_;
};

TEST_F(EvaluatorManagerTest, General) {
  std::string conf_file = "modules/prediction/testdata/adapter_conf.pb.txt";
  bool ret_load_conf =
      cyber::common::GetProtoFromFile(conf_file, &adapter_conf_);
  EXPECT_TRUE(ret_load_conf);
  EXPECT_TRUE(adapter_conf_.IsInitialized());

  ContainerManager container_manager;
  container_manager.Init(adapter_conf_);
  auto adc_trajectory_container =
      container_manager.GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  auto obstacles_container =
      container_manager.GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(obstacles_container);
  obstacles_container->Insert(perception_obstacles_);

  EvaluatorManager evaluator_manager;

  evaluator_manager.Init(prediction_conf_);
  evaluator_manager.Run(adc_trajectory_container,
      obstacles_container);

  Obstacle* obstacle_ptr = obstacles_container->GetObstacle(1);
  EXPECT_NE(obstacle_ptr, nullptr);
  const Feature& feature = obstacle_ptr->latest_feature();
  const LaneGraph& lane_graph = feature.lane().lane_graph();
  for (const auto& lane_sequence : lane_graph.lane_sequence()) {
    EXPECT_TRUE(lane_sequence.has_probability());
  }
}

}  // namespace prediction
}  // namespace apollo
