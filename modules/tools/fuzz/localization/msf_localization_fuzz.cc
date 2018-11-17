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

#include <iostream>
#include "modules/localization/msf/msf_localization.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/tools/fuzz/localization/proto/msf_localization_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace localization {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::localization::MSFLocalization;
using apollo::tools::fuzz::localization::MSFLocalizationFuzzMessage;

class MSFLocalizationFuzz {
 public:
  void Init();
  void Fuzz(MSFLocalizationFuzzMessage msf_localization_fuzz_message);

 private:
  std::unique_ptr<MSFLocalization> msf_localization_;
}msf_localization_fuzzer;

void MSFLocalizationFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::GPS);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_1->set_message_history_limit(50);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::IMU);
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_2->set_message_history_limit(50);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::RAW_IMU);
    sub_config_3->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_3->set_message_history_limit(50);
    auto *sub_config_4 = config.add_config();
    sub_config_4->set_type(AdapterConfig::POINT_CLOUD);
    sub_config_4->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_4->set_message_history_limit(5);
    auto *sub_config_5 = config.add_config();
    sub_config_5->set_type(AdapterConfig::LOCALIZATION);
    sub_config_5->set_mode(AdapterConfig::PUBLISH_ONLY);
    auto *sub_config_6 = config.add_config();
    sub_config_6->set_type(AdapterConfig::MONITOR);
    sub_config_6->set_mode(AdapterConfig::PUBLISH_ONLY);
    auto *sub_config_7 = config.add_config();
    sub_config_7->set_type(AdapterConfig::GNSS_RTK_OBS);
    sub_config_7->set_mode(AdapterConfig::RECEIVE_ONLY);
    auto *sub_config_8 = config.add_config();
    sub_config_8->set_type(AdapterConfig::GNSS_RTK_EPH);
    sub_config_8->set_mode(AdapterConfig::RECEIVE_ONLY);
    auto *sub_config_9 = config.add_config();
    sub_config_9->set_type(AdapterConfig::GNSS_BEST_POSE);
    sub_config_9->set_mode(AdapterConfig::RECEIVE_ONLY);
    auto *sub_config_10 = config.add_config();
    sub_config_10->set_type(AdapterConfig::LOCALIZATION_MSF_GNSS);
    sub_config_10->set_mode(AdapterConfig::PUBLISH_ONLY);
    auto *sub_config_11 = config.add_config();
    sub_config_11->set_type(AdapterConfig::LOCALIZATION_MSF_LIDAR);
    sub_config_11->set_mode(AdapterConfig::PUBLISH_ONLY);
    auto *sub_config_12 = config.add_config();
    sub_config_12->set_type(AdapterConfig::LOCALIZATION_MSF_SINS_PVA);
    sub_config_12->set_mode(AdapterConfig::PUBLISH_ONLY);
    auto *sub_config_13 = config.add_config();
    sub_config_13->set_type(AdapterConfig::LOCALIZATION_MSF_STATUS);
    sub_config_13->set_mode(AdapterConfig::PUBLISH_ONLY);
  }
  AdapterManager::Init(config);
  FLAGS_map_dir = "/apollo/modules/map/data/sunnyvale_big_loop";
  msf_localization_.reset(new MSFLocalization());
  msf_localization_->Start();
}

void MSFLocalizationFuzz::Fuzz(
    MSFLocalizationFuzzMessage msf_localization_fuzz_message) {
  msf_localization_->OnRawImu(msf_localization_fuzz_message.imu());
  msf_localization_->OnGnssBestPose(
      msf_localization_fuzz_message.gnss_best_pose());
  msf_localization_->OnGnssRtkObs(
      msf_localization_fuzz_message.gnss_rtk_obs());
  msf_localization_->OnGnssRtkEph(
      msf_localization_fuzz_message.gnss_rtk_eph());
}

// Before fuzzing, need to comment out
// "tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);"
// in msf_localization.cc
DEFINE_PROTO_FUZZER(
    const MSFLocalizationFuzzMessage& msf_localization_fuzz_message) {
  std::cout.setstate(std::ios_base::failbit);
  std::cerr.setstate(std::ios_base::failbit);
  msf_localization_fuzzer.Fuzz(msf_localization_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  msf_localization_fuzzer.Init();
  return 0;
}

}  // namespace localization
}  // namespace apollo
