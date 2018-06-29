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

#include <string>
#include "gtest/gtest.h"

#include "modules/common/util/file.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/prediction/common/kml_map_based_test.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterConfig;

class EvaluatorManagerTest : public KMLMapBasedTest {
 public:
  virtual void SetUp() {
    std::string file =
        "modules/prediction/testdata/single_perception_vehicle_onlane.pb.txt";
    CHECK(apollo::common::util::GetProtoFromFile(file, &perception_obstacles_));
  }

 protected:
  apollo::perception::PerceptionObstacles perception_obstacles_;
  common::adapter::AdapterManagerConfig adapter_conf_;
  PredictionConf prediction_conf_;
};

TEST_F(EvaluatorManagerTest, General) {
  std::string conf_file = "modules/prediction/testdata/adapter_conf.pb.txt";
  bool ret_load_conf = common::util::GetProtoFromFile(
      conf_file, &adapter_conf_);
  EXPECT_TRUE(ret_load_conf);
  EXPECT_TRUE(adapter_conf_.IsInitialized());

  ContainerManager::instance()->Init(adapter_conf_);
  ObstaclesContainer* obstacles_container = dynamic_cast<ObstaclesContainer*>(
      ContainerManager::instance()->GetContainer(
          AdapterConfig::PERCEPTION_OBSTACLES));
  CHECK_NOTNULL(obstacles_container);
  obstacles_container->Insert(perception_obstacles_);

  EvaluatorManager::instance()->Init(prediction_conf_);
  EvaluatorManager::instance()->Run(perception_obstacles_);

  Obstacle* obstacle_ptr = obstacles_container->GetObstacle(1);
  EXPECT_TRUE(obstacle_ptr != nullptr);
  const Feature& feature = obstacle_ptr->latest_feature();
  const LaneGraph& lane_graph = feature.lane().lane_graph();
  for (const auto& lane_sequence : lane_graph.lane_sequence()) {
    EXPECT_TRUE(lane_sequence.has_probability());
  }
}

}  // namespace prediction
}  // namespace apollo
