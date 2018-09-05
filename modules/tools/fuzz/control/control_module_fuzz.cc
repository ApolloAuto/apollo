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

#include "modules/control/control.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/control/common/control_gflags.h"
#include "modules/tools/fuzz/control/proto/control_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace control {

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::canbus::Chassis;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using apollo::tools::fuzz::control::ControlModuleFuzzMessage;

class ControlModuleFuzz {
 public:
  void Init();
  void Fuzz(ControlModuleFuzzMessage message);

 private:
  std::unique_ptr<Control> control_;
} control_module_fuzzer;

void ControlModuleFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::LOCALIZATION);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_1->set_message_history_limit(10);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::PAD);
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_2->set_message_history_limit(10);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::PLANNING_TRAJECTORY);
    sub_config_3->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_3->set_message_history_limit(10);
    auto *sub_config_4 = config.add_config();
    sub_config_4->set_type(AdapterConfig::CHASSIS);
    sub_config_4->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_4->set_message_history_limit(10);
    auto *sub_config_5 = config.add_config();
    sub_config_5->set_type(AdapterConfig::CONTROL_COMMAND);
    sub_config_5->set_mode(AdapterConfig::PUBLISH_ONLY);
    auto *sub_config_6 = config.add_config();
    sub_config_6->set_type(AdapterConfig::MONITOR);
    sub_config_6->set_mode(AdapterConfig::DUPLEX);
    sub_config_6->set_message_history_limit(1);
  }
  AdapterManager::Init(config);
  control_.reset(new Control());
  control_->Init();
  control_->Start();
}

void ControlModuleFuzz::Fuzz(ControlModuleFuzzMessage message) {
  control_->OnMonitor(message.monitor_message());
  control_->OnPad(message.pad_message());
  AdapterManager::FeedLocalizationData(
      message.localization_estimate());
  AdapterManager::FeedPlanningData(message.adc_trajectory());
  AdapterManager::FeedChassisData(message.chassis());
  ControlCommand control_command;
  if (message.adc_trajectory().trajectory_point().size() > 0) {
    control_->ProduceControlCommand(&control_command);
  }
  // AdapterManager::GetMonitor()->receive_callbacks_.clear();
}

DEFINE_PROTO_FUZZER(const ControlModuleFuzzMessage& message) {
  control_module_fuzzer.Fuzz(message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  control_module_fuzzer.Init();
  return 0;
}

}  // namespace control
}  // namespace apollo
