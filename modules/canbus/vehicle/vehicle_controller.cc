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

#include "modules/canbus/vehicle/vehicle_controller.h"

#include "cyber/common/log.h"

namespace apollo {
namespace canbus {

using common::ErrorCode;
using control::ControlCommand;

Chassis::DrivingMode VehicleController::driving_mode() {
  std::lock_guard<std::mutex> lock(mode_mutex_);
  return driving_mode_;
}

void VehicleController::set_driving_mode(
    const Chassis::DrivingMode &driving_mode) {
  std::lock_guard<std::mutex> lock(mode_mutex_);
  driving_mode_ = driving_mode;
}

ErrorCode VehicleController::SetDrivingMode(
    const Chassis::DrivingMode &driving_mode) {
  if (driving_mode == Chassis::EMERGENCY_MODE) {
    AINFO << "Can't set vehicle to EMERGENCY_MODE driving mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  // vehicle in emergency mode only response to manual mode to reset.
  if (driving_mode_ == Chassis::EMERGENCY_MODE &&
      driving_mode != Chassis::COMPLETE_MANUAL) {
    AINFO
        << "Vehicle in EMERGENCY_MODE, only response to COMPLETE_MANUAL mode.";
    AINFO << "Only response to RESET ACTION.";
    return ErrorCode::CANBUS_ERROR;
  }

  // if current mode is same as previous, no need to set.
  if (driving_mode_ == driving_mode) {
    return ErrorCode::OK;
  }

  switch (driving_mode) {
    case Chassis::COMPLETE_AUTO_DRIVE: {
      if (EnableAutoMode() != ErrorCode::OK) {
        AERROR << "Fail to EnableAutoMode.";
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    case Chassis::COMPLETE_MANUAL: {
      if (DisableAutoMode() != ErrorCode::OK) {
        AERROR << "Fail to DisableAutoMode.";
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    case Chassis::AUTO_STEER_ONLY: {
      if (EnableSteeringOnlyMode() != ErrorCode::OK) {
        AERROR << "Fail to EnableSpeedOnlyMode.";
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    case Chassis::AUTO_SPEED_ONLY: {
      if (EnableSpeedOnlyMode() != ErrorCode::OK) {
        AERROR << "Fail to EnableSpeedOnlyMode";
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    default:
      break;
  }
  return ErrorCode::OK;
}

ErrorCode VehicleController::Update(const ControlCommand &command) {
  if (!is_initialized_) {
    AERROR << "Controller not initialized.";
    return ErrorCode::CANBUS_ERROR;
  }

  ControlCommand control_command;
  control_command.CopyFrom(command);

  // execute action to tranform driving mode
  if (control_command.has_pad_msg() && control_command.pad_msg().has_action()) {
    AINFO << "Canbus received pad msg:"
          << control_command.pad_msg().ShortDebugString();
    Chassis::DrivingMode mode = Chassis::COMPLETE_MANUAL;
    switch (control_command.pad_msg().action()) {
      case control::DrivingAction::START: {
        mode = Chassis::COMPLETE_AUTO_DRIVE;
        break;
      }
      case control::DrivingAction::STOP:
      case control::DrivingAction::RESET: {
        mode = Chassis::COMPLETE_MANUAL;
        break;
      }
      default: {
        AERROR << "No response for this action.";
        break;
      }
    }
    SetDrivingMode(mode);
  }

  if (driving_mode_ == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode_ == Chassis::AUTO_SPEED_ONLY) {
    Gear(control_command.gear_location());
    Throttle(control_command.throttle());
    Brake(control_command.brake());
    SetEpbBreak(control_command);
    SetLimits();
  }

  if (driving_mode_ == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode_ == Chassis::AUTO_STEER_ONLY) {
    const double steering_rate_threshold = 1.0;
    if (control_command.steering_rate() > steering_rate_threshold) {
      Steer(control_command.steering_target(), control_command.steering_rate());
    } else {
      Steer(control_command.steering_target());
    }
  }

  if ((driving_mode_ == Chassis::COMPLETE_AUTO_DRIVE ||
       driving_mode_ == Chassis::AUTO_SPEED_ONLY ||
       driving_mode_ == Chassis::AUTO_STEER_ONLY) &&
      control_command.has_signal()) {
    SetHorn(control_command);
    SetTurningSignal(control_command);
    SetBeam(control_command);
  }

  return ErrorCode::OK;
}

}  // namespace canbus
}  // namespace apollo
