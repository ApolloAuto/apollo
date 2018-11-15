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

#include "modules/third_party_perception/third_party_perception.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/tools/fuzz/third_party_perception/proto/third_party_perception_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace third_party_perception {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::tools::fuzz::third_party_perception
    ::ThirdPartyPerceptionFuzzMessage;

class ThirdPartyPerceptionFuzz {
 public:
  void Init();
  void Fuzz(
      ThirdPartyPerceptionFuzzMessage third_party_perception_fuzz_message);

 private:
  std::unique_ptr<ThirdPartyPerception> third_party_perception_;
} third_party_perception_fuzzer;

void ThirdPartyPerceptionFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::MOBILEYE);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::DELPHIESR);
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::CONTI_RADAR);
    sub_config_3->set_mode(AdapterConfig::RECEIVE_ONLY);
    auto *sub_config_4 = config.add_config();
    sub_config_4->set_type(AdapterConfig::LOCALIZATION);
    sub_config_4->set_mode(AdapterConfig::RECEIVE_ONLY);
    auto *sub_config_5 = config.add_config();
    sub_config_5->set_type(AdapterConfig::PERCEPTION_OBSTACLES);
    sub_config_5->set_mode(AdapterConfig::PUBLISH_ONLY);
    auto *sub_config_6 = config.add_config();
    sub_config_6->set_type(AdapterConfig::CHASSIS);
    sub_config_6->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_6->set_message_history_limit(1);
  }
  AdapterManager::Init(config);
  third_party_perception_.reset(new ThirdPartyPerception());
  FLAGS_adapter_config_filename =
      "modules/third_party_perception/conf/adapter.conf";
  third_party_perception_->Init();
}

void ThirdPartyPerceptionFuzz::Fuzz(
    ThirdPartyPerceptionFuzzMessage third_party_perception_fuzz_message) {
  third_party_perception_->OnMobileye(
      third_party_perception_fuzz_message.mobileye());
  third_party_perception_->OnChassis(
      third_party_perception_fuzz_message.chassis());
  third_party_perception_->OnDelphiESR(
      third_party_perception_fuzz_message.delphi_esr());
  third_party_perception_->OnContiRadar(
      third_party_perception_fuzz_message.conti_radar());
  third_party_perception_->OnLocalization(
      third_party_perception_fuzz_message.localization_estimate());
  ros::TimerEvent event;
  third_party_perception_->OnTimer(event);
}

DEFINE_PROTO_FUZZER(
    const ThirdPartyPerceptionFuzzMessage& third_party_perception_fuzz_message
  ) {
  third_party_perception_fuzzer.Fuzz(third_party_perception_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  third_party_perception_fuzzer.Init();
  return 0;
}

}  // namespace third_party_perception
}  // namespace apollo
