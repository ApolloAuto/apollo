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

#define private public

#include "modules/prediction/prediction.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/tools/fuzz/prediction/proto/prediction_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::tools::fuzz::prediction::PredictionFuzzMessage;

class PredictionFuzz {
 public:
  void Init();
  void Fuzz(PredictionFuzzMessage prediction_fuzz_message);

 private:
  std::unique_ptr<Prediction> prediction_;
} prediction_fuzzer;

void PredictionFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::PERCEPTION_OBSTACLES);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_1->set_message_history_limit(1);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::LOCALIZATION);
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_2->set_message_history_limit(10);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::PLANNING_TRAJECTORY);
    sub_config_3->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_3->set_message_history_limit(1);
    auto *sub_config_4 = config.add_config();
    sub_config_4->set_type(AdapterConfig::RELATIVE_MAP);
    sub_config_4->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_4->set_message_history_limit(1);
    auto *sub_config_5 = config.add_config();
    sub_config_5->set_type(AdapterConfig::PREDICTION);
    sub_config_5->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config_5->set_message_history_limit(10);
  }
  AdapterManager::Init(config);
  prediction_.reset(new Prediction());
  prediction_->Init();
}

void PredictionFuzz::Fuzz(PredictionFuzzMessage prediction_fuzz_message) {
  prediction_->OnLocalization(
      prediction_fuzz_message.localization_estimate());
  prediction_->OnPlanning(prediction_fuzz_message.adc_trajectory());
  prediction_->RunOnce(
      prediction_fuzz_message.perception_obstacles());
}

DEFINE_PROTO_FUZZER(const PredictionFuzzMessage& prediction_fuzz_message) {
  prediction_fuzzer.Fuzz(prediction_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  prediction_fuzzer.Init();
  return 0;
}

}  // namespace prediction
}  // namespace apollo
