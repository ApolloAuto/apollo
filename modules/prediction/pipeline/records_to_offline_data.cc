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

#include "cyber/common/file.h"

#include "absl/strings/str_split.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/message_process.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/proto/prediction_conf.pb.h"
#include "modules/common/util/data_extraction.h"

namespace apollo {
namespace prediction {

void GenerateDataForLearning() {
  apollo::hdmap::HDMapUtil::ReloadMaps();
  if (!FeatureOutput::Ready()) {
    AERROR << "Feature output is not ready.";
    return;
  }
  if (FLAGS_prediction_offline_bags.empty()) {
    return;
  }

  PredictionConf prediction_conf;
  if (!cyber::common::GetProtoFromFile(FLAGS_prediction_conf_file,
                                       &prediction_conf)) {
    AERROR << "Unable to load adapter conf file: "
           << FLAGS_prediction_adapter_config_filename;
    return;
  }
  ADEBUG << "Adapter config file is loaded into: "
         << prediction_conf.ShortDebugString();

  auto container_manager = std::make_shared<ContainerManager>();
  EvaluatorManager evaluator_manager;
  PredictorManager predictor_manager;
  ScenarioManager scenario_manager;

  if (!MessageProcess::Init(container_manager.get(), &evaluator_manager,
                            &predictor_manager, prediction_conf)) {
    return;
  }
  const std::vector<std::string> inputs =
      absl::StrSplit(FLAGS_prediction_offline_bags, ':');
  for (const auto& input : inputs) {
    std::vector<std::string> offline_bags;
    GetRecordFileNames(boost::filesystem::path(input), &offline_bags);
    std::sort(offline_bags.begin(), offline_bags.end());
    AINFO << "For input " << input << ", found " << offline_bags.size()
          << "  rosbags to process";
    for (std::size_t i = 0; i < offline_bags.size(); ++i) {
      AINFO << "\tProcessing: [ " << i << " / " << offline_bags.size()
            << " ]: " << offline_bags[i];
      MessageProcess::ProcessOfflineData(prediction_conf, container_manager,
                                         &evaluator_manager, &predictor_manager,
                                         &scenario_manager, offline_bags[i]);
    }
  }
  FeatureOutput::Close();
}

}  // namespace prediction
}  // namespace apollo

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::prediction::GenerateDataForLearning();
  return 0;
}
