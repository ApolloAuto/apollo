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
#include "modules/guardian/guardian.h"

#include <cmath>

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/guardian/common/guardian_gflags.h"
#include "ros/include/ros/ros.h"

namespace apollo {
namespace guardian {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::control::ControlCommand;
using apollo::guardian::GuardianCommand;
using apollo::monitor::SystemStatus;

std::string Guardian::Name() const { return FLAGS_module_name; }

Status Guardian::Init() {
  AdapterManager::Init(FLAGS_adapter_config_filename);
  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";
  AdapterManager::AddChassisCallback(&Guardian::OnChassis, this);
  CHECK(AdapterManager::GetSystemStatus())
      << "SystemStatus is not initialized.";
  AdapterManager::AddSystemStatusCallback(&Guardian::OnSystemStatus, this);
  CHECK(AdapterManager::GetControlCommand()) << "Control is not initialized.";
  AdapterManager::AddControlCommandCallback(&Guardian::OnControl, this);
  return Status::OK();
}

Status Guardian::Start() {
  const double duration = 1.0 / FLAGS_guardian_cmd_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &Guardian::OnTimer, this);

  return Status::OK();
}

void Guardian::Stop() { timer_.stop(); }

void Guardian::OnTimer(const ros::TimerEvent&) {
  ADEBUG << "Timer is triggered: publish Guardian result";
  std::lock_guard<std::mutex> lock(mutex_);
  if (!system_status_.has_safety_mode_trigger_time()) {
    ADEBUG << "Safety mode not triggerd, bypass control command";
    ByPassControlCommand();
  } else {
    ADEBUG << "Safety mode triggerd, enable safty mode";
    TriggerSafetyMode();
  }
  AdapterManager::FillGuardianHeader(FLAGS_node_name, &guardian_cmd_);
  AdapterManager::PublishGuardian(guardian_cmd_);
}

void Guardian::OnChassis(const Chassis& message) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  chassis_.CopyFrom(message);
}

void Guardian::OnSystemStatus(const SystemStatus& message) {
  ADEBUG << "Received monitor data: run monitor callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  system_status_.CopyFrom(message);
}

void Guardian::OnControl(const ControlCommand& message) {
  ADEBUG << "Received control data: run control command callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  control_cmd_.CopyFrom(message);
}

void Guardian::ByPassControlCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  guardian_cmd_.CopyFrom(control_cmd_);
}

void Guardian::TriggerSafetyMode() {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  guardian_cmd_.set_throttle(0.0);
  guardian_cmd_.set_brake(FLAGS_guardian_cmd_soft_stop_percentage);
  guardian_cmd_.set_steering_target(0.0);
  guardian_cmd_.set_steering_rate(0.0);
  guardian_cmd_.set_is_in_safe_mode(true);
}

}  // namespace guardian
}  // namespace apollo
