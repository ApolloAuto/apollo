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
#include "libfuzzer/libfuzzer_macro.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterConfig;
using apollo::perception::PerceptionObstacles;

class EvaluatorManagerFuzz {
 public:
  void target(PerceptionObstacles obstacles);

 protected:
  common::adapter::AdapterManagerConfig adapter_conf_;
  PredictionConf prediction_conf_;
}evaluator_manager_fuzzer;

void EvaluatorManagerFuzz::target(PerceptionObstacles obstacles) {
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
  obstacles_container->Insert(obstacles);

  EvaluatorManager::instance()->Init(prediction_conf_);
  EvaluatorManager::instance()->Run(obstacles);

  Obstacle* obstacle_ptr = obstacles_container->GetObstacle(1);
  if (obstacle_ptr) {
    const Feature& feature = obstacle_ptr->latest_feature();
    const LaneGraph& lane_graph = feature.lane().lane_graph();
    for (const auto& lane_sequence : lane_graph.lane_sequence()) {
      EXPECT_TRUE(lane_sequence.has_probability());
  }
}
}

}  // namespace prediction
}  // namespace apollo

DEFINE_PROTO_FUZZER(const apollo::perception::PerceptionObstacles& message) {
  apollo::prediction::evaluator_manager_fuzzer.target(message);
}
