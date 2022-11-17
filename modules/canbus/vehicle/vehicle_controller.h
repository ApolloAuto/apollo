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

/**
 * @file vehicle_controller.h
 * @brief The class of VehicleController
 */

#pragma once

#include <unordered_map>

#include "modules/canbus/proto/canbus_conf.pb.h"

#include "cyber/common/log.h"
#include "modules/common_msgs/basic_msgs/error_code.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

using ::apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::MessageManager;

/**
 * @class VehicleController
 *
 * @brief This is the interface class of vehicle controller. It defines pure
 * virtual functions, and also some implemented common functions.
 */
template <typename SensorType>
class VehicleController {
 public:
  virtual ~VehicleController() = default;

  /**
   * @brief initialize the vehicle controller.
   * @param can_sender a pointer to canbus sender.
   * @param message_manager a pointer to the message_manager.
   * @return error_code
   */
  virtual common::ErrorCode Init(
      const VehicleParameter &params, CanSender<SensorType> *const can_sender,
      MessageManager<SensorType> *const message_manager) = 0;

  /**
   * @brief start the vehicle controller.
   * @return true if successfully started.
   */
  virtual bool Start() = 0;

  /**
   * @brief stop the vehicle controller.
   */
  virtual void Stop() = 0;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  virtual Chassis chassis() = 0;

  /**
   * @brief update the vehicle controller.
   * @param command the control command
   * @return error_code
   */
  virtual common::ErrorCode Update(const control::ControlCommand &command);

  /**
   * @brief set vehicle to appointed driving mode.
   * @param driving mode to be appointed.
   * @return error_code
   */
  virtual common::ErrorCode SetDrivingMode(
      const Chassis::DrivingMode &driving_mode);

 private:
  /*
   * @brief main logical function for operation the car enter or exit the auto
   * driving
   */
  virtual void Emergency() = 0;

  virtual common::ErrorCode EnableAutoMode() = 0;
  virtual common::ErrorCode DisableAutoMode() = 0;
  virtual common::ErrorCode EnableSteeringOnlyMode() = 0;
  virtual common::ErrorCode EnableSpeedOnlyMode() = 0;

  /*
   * @brief NEUTRAL, REVERSE, DRIVE
   */
  virtual void Gear(Chassis::GearPosition state) = 0;

  /*
   * @brief detail function for auto driving brake with new acceleration
   * acceleration:0.00~99.99, unit:%
   */
  virtual void Brake(double acceleration) = 0;

  /*
   * @brief drive with old acceleration gas:0.00~99.99 unit:%
   */
  virtual void Throttle(double throttle) = 0;

  /*
   * @brief drive with new acceleration/deceleration:-7.0~7.0, unit:m/s^2,
   * acc:-7.0~7.0, unit:m/s^2
   */
  virtual void Acceleration(double acc) = 0;

  /*
   * @brief steering with old angle speed angle:-99.99~0.00~99.99, unit:%,
   * left:+, right:-
   */
  virtual void Steer(double angle) = 0;

  /*
   * @brief steering with new angle speed angle:-99.99~0.00~99.99, unit:%,
   * left:+, right:- angle_spd:0.00~99.99, unit:deg/s
   */
  virtual void Steer(double angle, double angle_spd) = 0;

  /*
   * @brief set Electrical Park Brake
   */
  virtual void SetEpbBreak(const control::ControlCommand &command) = 0;
  virtual void SetBeam(const control::ControlCommand &command) = 0;
  virtual void SetHorn(const control::ControlCommand &command) = 0;
  virtual void SetTurningSignal(const control::ControlCommand &command) = 0;

  virtual void SetLimits() {}

  /**
   * @brief Response to vehicle ID request.
   * @return true if vehicle ID is successfully responsed.
   */
  virtual bool VerifyID() = 0;

 protected:
  virtual Chassis::DrivingMode driving_mode();
  virtual void set_driving_mode(const Chassis::DrivingMode &driving_mode);

 protected:
  canbus::VehicleParameter params_;
  common::VehicleParam vehicle_params_;
  CanSender<SensorType> *can_sender_ = nullptr;
  MessageManager<SensorType> *message_manager_ = nullptr;
  bool is_initialized_ = false;  // own by derviative concrete controller
  Chassis::DrivingMode driving_mode_ = Chassis::COMPLETE_MANUAL;
  bool is_reset_ = false;  // reset command from control command
  std::mutex mode_mutex_;  // only use in this base class
};

using common::ErrorCode;
using control::ControlCommand;

template <typename SensorType>
Chassis::DrivingMode VehicleController<SensorType>::driving_mode() {
  std::lock_guard<std::mutex> lock(mode_mutex_);
  return driving_mode_;
}

template <typename SensorType>
void VehicleController<SensorType>::set_driving_mode(
    const Chassis::DrivingMode &driving_mode) {
  std::lock_guard<std::mutex> lock(mode_mutex_);
  driving_mode_ = driving_mode;
}

template <typename SensorType>
ErrorCode VehicleController<SensorType>::SetDrivingMode(
    const Chassis::DrivingMode &driving_mode) {
  if (driving_mode == Chassis::EMERGENCY_MODE) {
    AINFO << "Can't set vehicle to EMERGENCY_MODE driving mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  // vehicle in emergency mode only response to manual mode to reset.
  if (this->driving_mode() == Chassis::EMERGENCY_MODE &&
      driving_mode != Chassis::COMPLETE_MANUAL) {
    AINFO
        << "Vehicle in EMERGENCY_MODE, only response to COMPLETE_MANUAL mode.";
    AINFO << "Only response to RESET ACTION.";
    return ErrorCode::CANBUS_ERROR;
  }

  // if current mode is same as previous, no need to set.
  if (this->driving_mode() == driving_mode) {
    return ErrorCode::OK;
  }

  switch (driving_mode) {
    case Chassis::COMPLETE_AUTO_DRIVE: {
      if (EnableAutoMode() != ErrorCode::OK) {
        AERROR << "Failed to enable auto mode.";
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    case Chassis::COMPLETE_MANUAL: {
      if (DisableAutoMode() != ErrorCode::OK) {
        AERROR << "Failed to disable auto mode.";
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    case Chassis::AUTO_STEER_ONLY: {
      if (EnableSteeringOnlyMode() != ErrorCode::OK) {
        AERROR << "Failed to enable steer only mode.";
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    case Chassis::AUTO_SPEED_ONLY: {
      if (EnableSpeedOnlyMode() != ErrorCode::OK) {
        AERROR << "Failed to enable speed only mode";
        return ErrorCode::CANBUS_ERROR;
      }
      break;
    }
    default:
      break;
  }
  return ErrorCode::OK;
}

template <typename SensorType>
ErrorCode VehicleController<SensorType>::Update(
    const ControlCommand &control_command) {
  if (!is_initialized_) {
    AERROR << "Controller is not initialized.";
    return ErrorCode::CANBUS_ERROR;
  }

  // Execute action to transform driving mode
  if (control_command.has_pad_msg() && control_command.pad_msg().has_action()) {
    AINFO << "Canbus received pad msg: "
          << control_command.pad_msg().ShortDebugString();
    if (control_command.pad_msg().action() == control::DrivingAction::VIN_REQ) {
      if (!VerifyID()) {
        AINFO << "Response vid failed, please request again.";
      } else {
        AINFO << "Response vid success!";
      }
    } else {
      Chassis::DrivingMode mode = Chassis::COMPLETE_MANUAL;
      switch (control_command.pad_msg().action()) {
        case control::DrivingAction::START: {
          mode = Chassis::COMPLETE_AUTO_DRIVE;
          break;
        }
        case control::DrivingAction::STOP:
        case control::DrivingAction::RESET: {
          // In COMPLETE_MANUAL mode
          break;
        }
        default: {
          AERROR << "No response for this action.";
          break;
        }
      }
      auto error_code = SetDrivingMode(mode);
      if (error_code != ErrorCode::OK) {
        AERROR << "Failed to set driving mode.";
      } else {
        AINFO << "Set driving mode success.";
      }
    }
  }

  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    Gear(control_command.gear_location());
    Throttle(control_command.throttle());
    Acceleration(control_command.acceleration());
    Brake(control_command.brake());
    SetEpbBreak(control_command);
    SetLimits();
  }

  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    const double steering_rate_threshold = 1.0;
    if (control_command.steering_rate() > steering_rate_threshold) {
      Steer(control_command.steering_target(), control_command.steering_rate());
    } else {
      Steer(control_command.steering_target());
    }
  }

  if ((driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
       driving_mode() == Chassis::AUTO_SPEED_ONLY ||
       driving_mode() == Chassis::AUTO_STEER_ONLY) &&
      control_command.has_signal()) {
    SetHorn(control_command);
    SetTurningSignal(control_command);
    SetBeam(control_command);
  }

  return ErrorCode::OK;
}

}  // namespace canbus
}  // namespace apollo
