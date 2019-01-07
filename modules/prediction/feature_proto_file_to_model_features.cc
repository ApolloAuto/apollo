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

#include <string>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/offline_features.pb.h"
#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/prediction/util/data_extraction.h"

using apollo::prediction::ContainerManager;
using apollo::prediction::EvaluatorManager;
using apollo::prediction::ObstaclesContainer;
using apollo::prediction::Obstacle;
using apollo::prediction::Feature;
using apollo::prediction::Features;
using apollo::prediction::PredictionConf;

void OfflineProcessFeatureProtoFile(
    const std::string& features_proto_file_name) {
  AERROR << "start";
  // Load prediction conf
  PredictionConf prediction_conf;
  if (!apollo::common::util::GetProtoFromFile(FLAGS_prediction_conf_file,
                                      &prediction_conf)) {
    AERROR << "Unable to load prediction conf file: "
           << FLAGS_prediction_conf_file;
    return;
  }
  ADEBUG << "Prediction config file is loaded into: "
            << prediction_conf.ShortDebugString();

  apollo::common::adapter::AdapterManagerConfig adapter_conf;
  if (!apollo::common::util::GetProtoFromFile(
  	FLAGS_prediction_adapter_config_filename, &adapter_conf)) {
    AERROR << "Unable to load adapter conf file: "
           << FLAGS_prediction_adapter_config_filename;
    return;
  }

  // Initialization of all managers
  ContainerManager::Instance()->Init(adapter_conf);
  EvaluatorManager::Instance()->Init(prediction_conf);

  auto obstacles_container_ptr = ContainerManager::Instance()->GetContainer<
      ObstaclesContainer>(
      	apollo::common::adapter::AdapterConfig::PERCEPTION_OBSTACLES);
  if (obstacles_container_ptr == nullptr) {
  	return;
  }
  obstacles_container_ptr->Clear();
  Features features;
  apollo::cyber::common::GetProtoFromBinaryFile(
      features_proto_file_name, &features);
  for (const Feature& feature : features.feature()) {
    obstacles_container_ptr->InsertFeatureProto(feature);
    Obstacle* obstacle_ptr = obstacles_container_ptr->GetObstacle(feature.id());
    EvaluatorManager::Instance()->EvaluateObstacle(obstacle_ptr);
  }
}

int main(int argc, char *argv[]) {
  // google::ParseCommandLineFlags(&argc, &argv, true);
  std::string file_name = "/apollo/data/prediction/feature.0.bin.junction.label";
  OfflineProcessFeatureProtoFile(file_name);
  return 0;
}
