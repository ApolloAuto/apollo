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

#include <glog/logging.h>
#include <chrono>
#include <mutex>
#include <thread>
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/control/common/control_gflags.h"
#include "modules/control/proto/pad_msg.pb.h"

using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::canbus::Chassis;

namespace apollo {
namespace control {
namespace {

class SwitchDrivingMode {
 public:
  static void EnterAutoMode() {
    // We always send a reset message before sending start.
    SendPadMessage(Chassis::COMPLETE_MANUAL, DrivingAction::RESET);
    WaitUntilManualMode();
    // delay for canbus reset
    sleep(1);
    SendPadMessage(Chassis::COMPLETE_AUTO_DRIVE, DrivingAction::START);
  }

  static void MonitorDrivingMode(const apollo::canbus::Chassis& status) {
    auto driving_mode = status.driving_mode();
    std::lock_guard<std::mutex> guard(current_mode_mutex_);
    if (driving_mode != current_mode_) {
      AINFO << "Driving mode changed: " << current_mode_ << " -> "
            << driving_mode;
      current_mode_ = driving_mode;
    }
  }

 private:
  static void SendPadMessage(Chassis::DrivingMode mode, DrivingAction action) {
    AINFO << "Sending PadMessage, DrivingMode=" << mode
          << ", DrivingAction=" << action;
    PadMessage pb;
    pb.set_driving_mode(mode);
    pb.set_action(action);
    AdapterManager::FillPadHeader("enter_auto_mode", pb.mutable_header());
    AdapterManager::PublishPad(pb);
  }

  static void WaitUntilManualMode() {
    while (true) {
      {
        // Lock and access current driving mode.
        std::lock_guard<std::mutex> guard(current_mode_mutex_);
        if (current_mode_ == Chassis::COMPLETE_MANUAL) {
          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  static std::mutex current_mode_mutex_;
  static Chassis::DrivingMode current_mode_;
};

// Init static members.
std::mutex SwitchDrivingMode::current_mode_mutex_;
Chassis::DrivingMode SwitchDrivingMode::current_mode_ =
    Chassis::COMPLETE_MANUAL;

}  // namespace
}  // namespace control
}  // namespace apollo

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_v = 4;

  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "enter_auto_mode");

  // Setup AdapterManager.
  AdapterManagerConfig config;
  config.set_is_ros(true);
  {
    auto* sub_config = config.add_config();
    sub_config->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config->set_type(AdapterConfig::PAD);
  }

  {
    auto* sub_config = config.add_config();
    sub_config->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config->set_type(AdapterConfig::CHASSIS);
  }
  AdapterManager::Init(config);
  AdapterManager::SetChassisCallback(
      apollo::control::SwitchDrivingMode::MonitorDrivingMode);

  // FIXME: delay for ros pub/sub ready
  sleep(1);
  apollo::control::SwitchDrivingMode::EnterAutoMode();
  return 0;
}
