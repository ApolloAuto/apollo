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

#include "modules/prediction/predictor/predictor_manager.h"

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
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "libfuzzer/libfuzzer_macro.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterConfig;
using apollo::perception::PerceptionObstacles;

protobuf_mutator::protobuf::LogSilencer log_silincer;

class PredictorManagerFuzz {
 public:
  void target(PerceptionObstacles obstacles);

 protected:
  common::adapter::AdapterManagerConfig adapter_conf_;
  PredictionConf prediction_conf_;
}predictor_manager_fuzzer;

void PredictorManagerFuzz::target(PerceptionObstacles obstacles) {
  FLAGS_enable_trim_prediction_trajectory = false;
  std::string conf_file = "modules/prediction/testdata/adapter_conf.pb.txt";
  bool ret_load_conf = common::util::GetProtoFromFile(
      conf_file, &adapter_conf_);
  EXPECT_TRUE(ret_load_conf);
  EXPECT_TRUE(adapter_conf_.IsInitialized());

  ContainerManager::instance()->Init(adapter_conf_);
  EvaluatorManager::instance()->Init(prediction_conf_);
  PredictorManager::instance()->Init(prediction_conf_);

  EvaluatorManager::instance()->Run(obstacles);
  PredictorManager::instance()->Run(obstacles);

  const PredictionObstacles& prediction_obstacles =
      PredictorManager::instance()->prediction_obstacles();
}

}  // namespace prediction
}  // namespace apollo

DEFINE_PROTO_FUZZER(const apollo::perception::PerceptionObstacles& message) {
  apollo::prediction::predictor_manager_fuzzer.target(message);
}
