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

#include "modules/guardian/guardian.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/guardian/common/guardian_gflags.h"
#include "modules/tools/fuzz/guardian/proto/guardian_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace guardian {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::tools::fuzz::guardian::GuardianFuzzMessage;

class GuardianFuzz {
 public:
  void Init();
  void Fuzz(GuardianFuzzMessage guardian_fuzz_message);

 private:
  std::unique_ptr<Guardian> guardian_;
} guardian_fuzzer;

void GuardianFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::CHASSIS);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_1->set_message_history_limit(1);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::SYSTEM_STATUS);
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_2->set_message_history_limit(1);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::CONTROL_COMMAND);
    sub_config_3->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_3->set_message_history_limit(1);
    auto *sub_config_4 = config.add_config();
    sub_config_4->set_type(AdapterConfig::GUARDIAN);
    sub_config_4->set_mode(AdapterConfig::PUBLISH_ONLY);
  }
  AdapterManager::Init(config);
  FLAGS_adapter_config_filename =
    "modules/tools/fuzz/guardian/conf/adapter.conf";
  FLAGS_guardian_enabled = true;
  guardian_.reset(new Guardian());
  guardian_->Init();
  guardian_->Start();
}

void GuardianFuzz::Fuzz(GuardianFuzzMessage guardian_fuzz_message) {
  AdapterManager::PublishChassis(guardian_fuzz_message.chassis());
  AdapterManager::PublishSystemStatus(guardian_fuzz_message.system_status());
  AdapterManager::PublishControlCommand(
      guardian_fuzz_message.control_command());
  ros::TimerEvent event;
  guardian_->OnTimer(event);
}

DEFINE_PROTO_FUZZER(const GuardianFuzzMessage& guardian_fuzz_message) {
  guardian_fuzzer.Fuzz(guardian_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  guardian_fuzzer.Init();
  return 0;
}

}  // namespace guardian
}  // namespace apollo
