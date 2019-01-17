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
using apollo::prediction::DataForLearning;
using apollo::prediction::ListDataForLearning;
using apollo::prediction::PredictionConf;

void OfflineProcessFeatureProtoFile(
    const std::string& input_features_proto_filename,
    const std::string& output_filename) {
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
  ListDataForLearning list_data_for_learning;
  Features features;
  apollo::cyber::common::GetProtoFromBinaryFile(
      input_features_proto_filename, &features);
  for (const Feature& feature : features.feature()) {
    obstacles_container_ptr->InsertFeatureProto(feature);
    Obstacle* obstacle_ptr = obstacles_container_ptr->GetObstacle(feature.id());
    EvaluatorManager::Instance()->EvaluateObstacle(obstacle_ptr);
    Feature latest_feature = obstacle_ptr->latest_feature();
    DataForLearning data_for_learning;
    data_for_learning.set_id(latest_feature.id());
    data_for_learning.set_timestamp(latest_feature.timestamp());
    if (FLAGS_extract_feature_type == "junction") {
      if (!latest_feature.has_junction_feature() ||
          latest_feature.junction_feature().junction_mlp_feature_size() == 0 ||
          latest_feature.junction_feature().junction_mlp_label_size() == 0) {
        continue;
      }
      auto junction_feature = latest_feature.junction_feature();
      for (int i = 0; i < junction_feature.junction_mlp_feature_size(); i++) {
        data_for_learning.add_features_for_learning(
            junction_feature.junction_mlp_feature(i));
      }
      for (int j = 0; j < junction_feature.junction_mlp_label_size(); j++) {
        data_for_learning.add_labels(
            junction_feature.junction_mlp_label(j));
      }
    } else if (FLAGS_extract_feature_type == "cruise") {
      if (latest_feature.lane().lane_graph().lane_sequence_size() == 0) {
        continue;
      }
      for (int i = 0; i < latest_feature.lane().
          lane_graph().lane_sequence_size(); i++) {
        auto curr_lane_sequence = latest_feature.lane().
            lane_graph().lane_sequence(i);
        if (curr_lane_sequence.features().mlp_features_size() == 0) {
          continue;
        }
        data_for_learning.add_features_for_learning(
            curr_lane_sequence.lane_sequence_id());
        for (int j = 0; j < curr_lane_sequence.features().mlp_features_size();
            j++) {
          data_for_learning.add_features_for_learning(
              curr_lane_sequence.features().mlp_features(j));
        }
        data_for_learning.add_labels(curr_lane_sequence.label());
        data_for_learning.add_labels(curr_lane_sequence.time_to_lane_edge());
        data_for_learning.add_labels(curr_lane_sequence.time_to_lane_center());
      }
      if (data_for_learning.labels_size() == 0) {
        continue;
      }
    }
    list_data_for_learning.add_data_for_learning()->CopyFrom(data_for_learning);
  }

  // write to file
  if (list_data_for_learning.data_for_learning_size() <= 0) {
    ADEBUG << "Skip writing empty data_for_learning.";
  } else {
    apollo::common::util::SetProtoToBinaryFile(list_data_for_learning,
                                               output_filename);
  }
}

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::string input_filename = FLAGS_offline_feature_proto_file_name;
  std::string output_filename = FLAGS_output_filename;
  std::string feature_type = FLAGS_extract_feature_type;
  OfflineProcessFeatureProtoFile(input_filename, output_filename);
  return 0;
}
