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

#include "modules/localization/rtk/rtk_localization.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/tools/fuzz/localization/proto/rtk_localization_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace localization {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::localization::LocalizationEstimate;
using apollo::localization::RTKLocalization;
using apollo::tools::fuzz::localization::RTKLocalizationFuzzMessage;

class RTKLocalizationFuzz {
 public:
  void Init();
  void Fuzz(RTKLocalizationFuzzMessage rtk_localization_fuzz_message);

 private:
  std::unique_ptr<RTKLocalization> rtk_localization_;
} rtk_localization_fuzzer;

void RTKLocalizationFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config_1->set_type(AdapterConfig::LOCALIZATION);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_2->set_type(AdapterConfig::IMU);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_3->set_type(AdapterConfig::GPS);
    auto *sub_config_4 = config.add_config();
    sub_config_4->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config_4->set_type(AdapterConfig::MONITOR);
  }
  AdapterManager::Init(config);
  rtk_localization_.reset(new RTKLocalization());
  rtk_localization_->Start();
}

void RTKLocalizationFuzz::Fuzz(
    RTKLocalizationFuzzMessage rtk_localization_fuzz_message) {
  AdapterManager::FeedGpsData(rtk_localization_fuzz_message.gps());
  AdapterManager::FeedImuData(rtk_localization_fuzz_message.imu());
  ros::TimerEvent event;
  rtk_localization_->OnTimer(event);
}

// Before fuzzing, need to comment out
// "tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);"
// in rtk_localization.cc
DEFINE_PROTO_FUZZER(
    const RTKLocalizationFuzzMessage& rtk_localization_fuzz_message) {
  rtk_localization_fuzzer.Fuzz(rtk_localization_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  rtk_localization_fuzzer.Init();
  return 0;
}

}  // namespace localization
}  // namespace apollo
