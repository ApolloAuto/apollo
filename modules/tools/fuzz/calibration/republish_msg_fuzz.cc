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

#define private public

#include "modules/calibration/republish_msg/republish_msg.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/calibration/republish_msg/common/republish_msg_gflags.h"
#include "modules/tools/fuzz/calibration/proto/republish_msg_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace calibration {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::tools::fuzz::calibration::RepublishMsgFuzzMessage;

class RepublishMsgFuzz {
 public:
  void Init();
  void Fuzz(RepublishMsgFuzzMessage republish_msg_fuzz_message);

 private:
  std::unique_ptr<RepublishMsg> republish_msg_;
} republish_msg_fuzzer;

void RepublishMsgFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::GPS);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_1->set_message_history_limit(100);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::INS_STAT);
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_2->set_message_history_limit(1);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::RELATIVE_ODOMETRY);
    sub_config_3->set_mode(AdapterConfig::PUBLISH_ONLY);
  }
  AdapterManager::Init(config);
  republish_msg_.reset(new RepublishMsg());
  republish_msg_->Init();
}

void RepublishMsgFuzz::Fuzz(
    RepublishMsgFuzzMessage republish_msg_fuzz_message) {
  republish_msg_->OnGps(republish_msg_fuzz_message.gps());
  republish_msg_->OnInsStat(republish_msg_fuzz_message.ins_stat());
}

DEFINE_PROTO_FUZZER(
    const RepublishMsgFuzzMessage& republish_msg_fuzz_message) {
  republish_msg_fuzzer.Fuzz(republish_msg_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  republish_msg_fuzzer.Init();
  return 0;
}

}  // namespace calibration
}  // namespace apollo
