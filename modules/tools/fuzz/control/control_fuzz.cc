/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.

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

#include <memory>
#include <string>
#include <utility>
#include <iostream>

#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "modules/common/log.h"

#define private public

#include "modules/control/control.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "libfuzzer/libfuzzer_macro.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/tools/fuzz/control/proto/control_message.pb.h"


using apollo::common::time::Clock;

protobuf_mutator::protobuf::LogSilencer log_silincer;

namespace apollo {
namespace control {

using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessage;
using apollo::control::ControlCommand;
using apollo::control::PadMessage;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using apollo::canbus::Chassis;

class ControlFuzz {
 public:
  void SetUp();
  bool FuzzTarget(PadMessage pad_message,
    LocalizationEstimate localization_message,
    ADCTrajectory planning_message,
    Chassis chassis_message);
  void target();

 private:
  ControlCommand control_command_;
  Control control_;
}control_fuzzer;

void ControlFuzz::SetUp() {
    ros::Time::init();
    FLAGS_control_conf_file = "modules/control/testdata/conf/lincoln.pb.txt";
    FLAGS_control_adapter_config_filename =
      "modules/control/testdata/conf/adapter.conf";
    FLAGS_is_control_test_mode = true;
}
bool ControlFuzz::FuzzTarget(
  PadMessage pad_message,
  LocalizationEstimate localization_message,
  ADCTrajectory planning_message,
  Chassis chassis_message) {
  if (!common::util::GetProtoFromFile(FLAGS_control_conf_file,
                                      &control_.control_conf_)) {
    AERROR << "Unable to load control conf file: " << FLAGS_control_conf_file;
    return false;
  }

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  // set controller
  if (!control_.controller_agent_.Init(&(control_.control_conf_)).ok()) {
    AERROR << "Control init controller failed! Stopping...";
    return false;
  }
  control_.controller_agent_.Reset();
  AdapterManager::Init(FLAGS_control_adapter_config_filename);

  control_.OnPad(pad_message);
  AdapterManager::FeedLocalizationData(localization_message);
  AdapterManager::FeedPlanningData(planning_message);
  AdapterManager::FeedChassisData(chassis_message);

  AdapterManager::Observe();

  auto err = control_.ProduceControlCommand(&control_command_);
  if (!err.ok()) {
    ADEBUG << "control ProduceControlCommand failed";
    return false;
  }
  return true;
}

}  // namespace control
}  // namespace apollo

/******************************************************************************
 * Entry point of the fuzz test. The control_message combines PadMessage,
 * LocalizationEstimate, ADCTrajectory, and Chassis messages together to
 * fuzz the control module. 
 *****************************************************************************/
DEFINE_PROTO_FUZZER(const apollo::tools::fuzz::control_message& message) {
  apollo::control::control_fuzzer.SetUp();
  if (message.has_planning() &&
    message.planning().header().module_name() == "planning"
    && message.planning().trajectory_point().size() > 0) {
    apollo::control::control_fuzzer.FuzzTarget(
      message.pad(),
      message.localization(),
      message.planning(),
      message.chassis());
  }
}
