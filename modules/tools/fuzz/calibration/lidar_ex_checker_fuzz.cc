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

#include "modules/calibration/lidar_ex_checker/lidar_ex_checker.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/calibration/lidar_ex_checker/common/lidar_ex_checker_gflags.h"
#include "modules/tools/fuzz/calibration/proto/lidar_ex_checker_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace calibration {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::tools::fuzz::calibration::LidarExCheckerFuzzMessage;

class LidarExCheckerFuzz {
 public:
  void Init();
  void Fuzz(LidarExCheckerFuzzMessage lidar_ex_checker_fuzz_message);

 private:
  std::unique_ptr<LidarExChecker> lidar_ex_checker_;
} lidar_ex_checker_fuzzer;

void LidarExCheckerFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::POINT_CLOUD);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_1->set_message_history_limit(10);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::GPS);
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_2->set_message_history_limit(100);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::INS_STAT);
    sub_config_3->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_3->set_message_history_limit(1);
  }
  AdapterManager::Init(config);
  lidar_ex_checker_.reset(new LidarExChecker());
  lidar_ex_checker_->Init();
}

void LidarExCheckerFuzz::Fuzz(
    LidarExCheckerFuzzMessage lidar_ex_checker_fuzz_message) {
  AdapterManager::PublishGps(lidar_ex_checker_fuzz_message.gps());
  AdapterManager::PublishInsStat(lidar_ex_checker_fuzz_message.ins_stat());
}

DEFINE_PROTO_FUZZER(
    const LidarExCheckerFuzzMessage& lidar_ex_checker_fuzz_message) {
  lidar_ex_checker_fuzzer.Fuzz(lidar_ex_checker_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  lidar_ex_checker_fuzzer.Init();
  return 0;
}

}  // namespace calibration
}  // namespace apollo
