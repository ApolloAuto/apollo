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
#include "modules/guardian/guardian_component.h"

#include <cmath>

#include "modules/common/log.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace guardian {

using apollo::common::ErrorCode;

bool GuardianComponent::Init() {
  if (!GetProtoConfig(&guardian_conf_)) {
    AERROR << "Unable to load canbus conf file: " << ConfigFilePath();
    return false;
  }

  chassis_reader_ = node_->CreateReader<Chassis>(
      guardian_conf_.chassis_channel(),
      [this](const std::shared_ptr<Chassis>& chassis) {
        ADEBUG << "Received chassis data: run chassis callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        chassis_.CopyFrom(*chassis);
      });

  control_cmd_reader_ = node_->CreateReader<ControlCommand>(
      guardian_conf_.control_command_channel(),
      [this](const std::shared_ptr<ControlCommand>& cmd) {
        ADEBUG << "Received monitor data: run monitor callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        control_cmd_.CopyFrom(*cmd);
      });

  system_status_reader_ = node_->CreateReader<SystemStatus>(
      guardian_conf_.system_status_channel(),
      [this](const std::shared_ptr<SystemStatus>& status) {
        ADEBUG << "Received control data: run control command callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        system_status_.CopyFrom(*status);
      });

  guardian_writer_ =
      node_->CreateWriter<GuardianCommand>(guardian_conf_.guardian_channel());

  return true;
}

bool GuardianComponent::Proc() {
  ADEBUG << "Timer is triggered: publish GuardianComponent result";
  bool safety_mode_triggered = false;
  if (guardian_conf_.guardian_enable()) {
    std::lock_guard<std::mutex> lock(mutex_);
    safety_mode_triggered = system_status_.has_safety_mode_trigger_time();
  }

  if (safety_mode_triggered) {
    ADEBUG << "Safety mode triggered, enable safty mode";
    TriggerSafetyMode();
  } else {
    ADEBUG << "Safety mode not triggered, bypass control command";
    PassThroughControlCommand();
  }

  common::util::FillHeader(node_->Name(), &guardian_cmd_);
  guardian_writer_->Write(std::make_shared<GuardianCommand>(guardian_cmd_));
  return true;
}

void GuardianComponent::PassThroughControlCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  guardian_cmd_.mutable_control_command()->CopyFrom(control_cmd_);
}

void GuardianComponent::TriggerSafetyMode() {
  AINFO << "Safety state triggered, with system safety mode trigger time : "
        << system_status_.safety_mode_trigger_time();
  std::lock_guard<std::mutex> lock(mutex_);
  bool sensor_malfunction = false, obstacle_detected = false;
  if (!chassis_.surround().sonar_enabled() ||
      chassis_.surround().sonar_fault()) {
    AINFO << "Ultrasonic sensor not enabled for faulted, will do emergency "
             "stop!";
    sensor_malfunction = true;
  } else {
    // TODO(QiL) : Load for config
    for (int i = 0; i < chassis_.surround().sonar_range_size(); ++i) {
      if ((chassis_.surround().sonar_range(i) > 0.0 &&
           chassis_.surround().sonar_range(i) < 2.5) ||
          chassis_.surround().sonar_range(i) > 30) {
        AINFO << "Object detected or ultrasonic sensor fault output, will do "
                 "emergency stop!";
        obstacle_detected = true;
      }
    }
  }

  guardian_cmd_.mutable_control_command()->set_throttle(0.0);
  guardian_cmd_.mutable_control_command()->set_steering_target(0.0);
  guardian_cmd_.mutable_control_command()->set_steering_rate(25.0);
  guardian_cmd_.mutable_control_command()->set_is_in_safe_mode(true);

  // TODO(QiL) : Remove this one once hardware re-alignment is done.
  sensor_malfunction = false;
  obstacle_detected = false;
  AINFO << "Temporarily ignore the ultrasonic sensor output during hardware "
           "re-alignment!";

  if (system_status_.require_emergency_stop() || sensor_malfunction ||
      obstacle_detected) {
    AINFO << "Emergency stop triggered! with system status from monitor as : "
          << system_status_.require_emergency_stop();
    guardian_cmd_.mutable_control_command()->set_brake(
        guardian_conf_.guardian_cmd_emergency_stop_percentage());
  } else {
    AINFO << "Soft stop triggered! with system status from monitor as : "
          << system_status_.require_emergency_stop();
    guardian_cmd_.mutable_control_command()->set_brake(
        guardian_conf_.guardian_cmd_soft_stop_percentage());
  }
}

}  // namespace guardian
}  // namespace apollo
