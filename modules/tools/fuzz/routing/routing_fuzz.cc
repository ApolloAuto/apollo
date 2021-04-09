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

#include "modules/routing/routing.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/tools/fuzz/routing/proto/routing_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace routing {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::tools::fuzz::routing::RoutingFuzzMessage;

class RoutingFuzz {
 public:
  void Init();
  void Fuzz(RoutingFuzzMessage routing_fuzz_message);

 private:
  std::unique_ptr<Routing> routing_;
} routing_fuzzer;

void RoutingFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::ROUTING_REQUEST);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_1->set_message_history_limit(10);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::ROUTING_RESPONSE);
    sub_config_2->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config_2->set_latch(true);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::MONITOR);
    sub_config_3->set_mode(AdapterConfig::DUPLEX);
    sub_config_3->set_message_history_limit(1);
  }
  AdapterManager::Init(config);
  routing_.reset(new Routing());
  routing_->Init();
}

void RoutingFuzz::Fuzz(RoutingFuzzMessage routing_fuzz_message) {
  routing_->OnRoutingRequest(routing_fuzz_message.routing_request());
}

DEFINE_PROTO_FUZZER(const RoutingFuzzMessage& routing_fuzz_message) {
  routing_fuzzer.Fuzz(routing_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  routing_fuzzer.Init();
  return 0;
}

}  // namespace routing
}  // namespace apollo
