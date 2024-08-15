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
#include "modules/common_msgs/basic_msgs/error_code.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/external_command_msgs/chassis_command.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

using apollo::cyber::Time;
using ::apollo::drivers::canbus::CanReceiver;
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
  virtual common::ErrorCode Update(
      const external_command::ChassisCommand &command);

  /**
   * @brief set vehicle to appointed driving mode.
   * @param driving mode to be appointed.
   * @return error_code
   */
  virtual common::ErrorCode SetDrivingMode(
      const Chassis::DrivingMode &driving_mode);

  virtual bool CheckChassisCommunicationError();

  virtual void AddSendMessage();

  virtual SensorType GetNewRecvChassisDetail();

  virtual SensorType GetNewSenderChassisDetail();

  virtual Chassis::DrivingMode driving_mode();

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
   * @brief drive with new speed:-xx.0~xx.0, unit:m/s
   */
  virtual void Speed(double speed) {}

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
  virtual void SetLimits() {}

  /*
   * @brief Handle user-defined chassis actions
   */
  virtual common::ErrorCode HandleCustomOperation(
      const external_command::ChassisCommand &command) = 0;

  /*
   * @brief Handle Apollo chassis actions
   */
  virtual void HandleVehicleSignal(const common::VehicleSignal &signal) {
    if (signal.has_high_beam() || signal.has_low_beam()) SetBeam(signal);
    if (signal.has_horn()) SetHorn(signal);
    if (signal.has_turn_signal()) SetTurningSignal(signal);
  }

  virtual void SetBeam(const common::VehicleSignal &signal) = 0;
  virtual void SetHorn(const common::VehicleSignal &signal) = 0;
  virtual void SetTurningSignal(const common::VehicleSignal &signal) = 0;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  virtual common::VehicleSignal ProcessCommandChange(
      const common::VehicleSignal &signal, common::VehicleSignal *last_command);

  /**
   * @brief Response to vehicle ID request.
   * @return true if vehicle ID is successfully responsed.
   */
  virtual bool VerifyID() = 0;

 protected:
  virtual void set_driving_mode(const Chassis::DrivingMode &driving_mode);

 protected:
  common::VehicleSignal last_chassis_command_;
  common::VehicleSignal last_control_command_;
  canbus::VehicleParameter params_;
  common::VehicleParam vehicle_params_;
  CanSender<SensorType> *can_sender_ = nullptr;
  CanReceiver<SensorType> *can_receiver_ = nullptr;
  MessageManager<SensorType> *message_manager_ = nullptr;
  bool is_initialized_ = false;  // own by derviative concrete controller
  Chassis::DrivingMode driving_mode_ = Chassis::COMPLETE_MANUAL;
  bool is_reset_ = false;  // reset command from control command
  std::mutex mode_mutex_;  // only use in this base class
  uint32_t lost_chassis_reveive_detail_count_ = 0;  // check chassis detail lost
  bool is_need_count_ = true;                       // check chassis detail lost
  size_t sender_data_size_previous_ = 0.0;       // check apollo sender preiod
  int64_t start_time_ = 0;                       // check apollo sender preiod
  bool is_chassis_communication_error_ = false;  // check chassis communication
};

using common::ErrorCode;
using control::ControlCommand;
using external_command::ChassisCommand;

template <typename SensorType>
bool VehicleController<SensorType>::CheckChassisCommunicationError() {
  SensorType chassis_detail_sender;
  if (message_manager_->GetSensorCheckSenderData(&chassis_detail_sender) !=
      ErrorCode::OK) {
    AERROR_EVERY(100) << "Get " << typeid(SensorType).name()
                      << " chassis receive detail failed.";
  }
  size_t sender_data_size = chassis_detail_sender.ByteSizeLong();
  ADEBUG << "check chassis detail sender_data_size is " << sender_data_size;
  int64_t end_time = 0;
  if ((sender_data_size_previous_ < 2) && (sender_data_size > 2)) {
    end_time = ::apollo::cyber::Time::Now().ToMicrosecond();
    ADEBUG << "end_time is " << end_time;
    if (start_time_ > 0) {
      const double sender_diff = (end_time - start_time_) * 1e-3;
      ADEBUG << "sender protocol preiod is " << sender_diff;
    }
  } else if ((sender_data_size_previous_ > 2) && (sender_data_size < 2)) {
    start_time_ = ::apollo::cyber::Time::Now().ToMicrosecond();
    ADEBUG << "start_time_ is " << start_time_;
  }
  sender_data_size_previous_ = sender_data_size;
  message_manager_->ClearSensorCheckSenderData();

  SensorType chassis_detail_receiver;
  if (message_manager_->GetSensorCheckRecvData(&chassis_detail_receiver) !=
      ErrorCode::OK) {
    AERROR_EVERY(100) << "Get " << typeid(SensorType).name()
                      << " chassis receive detail failed.";
  }
  ADEBUG << "chassis_detail_receiver is "
         << chassis_detail_receiver.ShortDebugString();
  size_t receiver_data_size = chassis_detail_receiver.ByteSizeLong();
  ADEBUG << "check chassis detail receiver_data_size is " << receiver_data_size;
  // check receiver data is null
  if (receiver_data_size < 2) {
    if (is_need_count_) {
      lost_chassis_reveive_detail_count_++;
    }
  } else {
    lost_chassis_reveive_detail_count_ = 0;
    is_need_count_ = true;
  }
  ADEBUG << "lost_chassis_reveive_detail_count_ is "
         << lost_chassis_reveive_detail_count_;
  // check receive data lost threshold is (100 * 10)ms
  if (lost_chassis_reveive_detail_count_ > 100) {
    is_need_count_ = false;
    is_chassis_communication_error_ = true;
    AERROR << typeid(SensorType).name()
           << " chassis detail is lost, please check the "
              "communication error.";
    message_manager_->ClearSensorCheckRecvData();
    message_manager_->ResetSendMessages();
    return true;
  } else {
    is_chassis_communication_error_ = false;
  }
  message_manager_->ClearSensorCheckRecvData();

  return false;
}

template <typename SensorType>
void VehicleController<SensorType>::AddSendMessage() {}

template <typename SensorType>
SensorType VehicleController<SensorType>::GetNewRecvChassisDetail() {
  SensorType receiver_chassis_detail;
  if (message_manager_->GetSensorRecvData(&receiver_chassis_detail) !=
      ErrorCode::OK) {
    AERROR_EVERY(100) << "Get " << typeid(SensorType).name()
                      << " chassis detail receiver failed.";
    return receiver_chassis_detail;
  }
  ADEBUG << "receiver_chassis_detail is "
         << receiver_chassis_detail.ShortDebugString();
  if (is_chassis_communication_error_) {
    message_manager_->ClearSensorRecvData();
  }
  return receiver_chassis_detail;
}

template <typename SensorType>
SensorType VehicleController<SensorType>::GetNewSenderChassisDetail() {
  SensorType sender_chassis_detail;
  if (message_manager_->GetSensorSenderData(&sender_chassis_detail) !=
      ErrorCode::OK) {
    AERROR_EVERY(100) << "Get " << typeid(SensorType).name()
                      << " chassis detail receiver failed.";
    return sender_chassis_detail;
  }
  message_manager_->GetSensorSenderData(&sender_chassis_detail);
  ADEBUG << "sender_chassis_detail is "
         << sender_chassis_detail.ShortDebugString();
  if (can_sender_->IsMessageClear()) {
    message_manager_->ClearSensorSenderData();
  }
  return sender_chassis_detail;
}

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

  if (control_command.has_pad_msg() && control_command.pad_msg().has_action()) {
    ADEBUG << "Canbus received pad msg: "
           << control_command.pad_msg().ShortDebugString();
    const double current_timestamp = Time::Now().ToSecond();
    // pad_msg_time_diff: s
    const double pad_msg_time_diff =
        current_timestamp - control_command.pad_msg().header().timestamp_sec();
    // Execute action to transform driving mode
    if ((FLAGS_chassis_debug_mode ||
         (pad_msg_time_diff < FLAGS_pad_msg_delay_interval)) &&
        !is_chassis_communication_error_) {
      if (control_command.pad_msg().action() ==
          control::DrivingAction::VIN_REQ) {
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
          case control::DrivingAction::RESET: {
            // In COMPLETE_MANUAL mode
            AINFO << "Into the Reset action.";
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
    } else {
      ADEBUG << "pad msg time out, current time interval is "
             << pad_msg_time_diff << " s, threshold is "
             << FLAGS_pad_msg_delay_interval << " s";
    }
  }

  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    Gear(control_command.gear_location());
    Throttle(control_command.throttle());
    Acceleration(control_command.acceleration());
    Speed(control_command.speed());
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
    HandleVehicleSignal(
        ProcessCommandChange(control_command.signal(), &last_control_command_));
  }

  return ErrorCode::OK;
}

template <typename SensorType>
ErrorCode VehicleController<SensorType>::Update(
    const ChassisCommand &chassis_command) {
  if (!is_initialized_) {
    AERROR << "Controller is not initialized.";
    return ErrorCode::CANBUS_ERROR;
  }

  if ((driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
       driving_mode() == Chassis::AUTO_SPEED_ONLY ||
       driving_mode() == Chassis::AUTO_STEER_ONLY) &&
      chassis_command.has_basic_signal()) {
    HandleVehicleSignal(ProcessCommandChange(chassis_command.basic_signal(),
                                             &last_chassis_command_));
  }

  if (chassis_command.has_custom_operation()) {
    auto result = HandleCustomOperation(chassis_command);
    if (result != ErrorCode::OK) {
      return result;
    }
  }

  return ErrorCode::OK;
}

template <typename SensorType>
common::VehicleSignal VehicleController<SensorType>::ProcessCommandChange(
    const common::VehicleSignal &vehicle_signal,
    common::VehicleSignal *last_command) {
  common::VehicleSignal vehicle_signal_end;
  if (vehicle_signal.has_high_beam()) {
    if ((last_command->has_high_beam() &&
         last_command->high_beam() != vehicle_signal.has_high_beam()) ||
        !last_command->has_high_beam()) {
      vehicle_signal_end.set_high_beam(vehicle_signal.high_beam());
    }
  }

  if (vehicle_signal.has_low_beam()) {
    if ((last_command->has_low_beam() &&
         last_command->low_beam() != vehicle_signal.has_low_beam()) ||
        !last_command->has_low_beam()) {
      vehicle_signal_end.set_low_beam(vehicle_signal.low_beam());
    }
  }

  if (vehicle_signal.has_horn()) {
    if ((last_command->has_horn() &&
         last_command->horn() != vehicle_signal.horn()) ||
        !last_command->horn()) {
      vehicle_signal_end.set_horn(vehicle_signal.horn());
    }
  }

  if (vehicle_signal.has_turn_signal()) {
    if ((last_command->has_turn_signal() &&
         last_command->turn_signal() != vehicle_signal.turn_signal()) ||
        !last_command->turn_signal()) {
      vehicle_signal_end.set_turn_signal(vehicle_signal.turn_signal());
    }
  }
  *last_command = vehicle_signal_end;
  return vehicle_signal_end;
}

}  // namespace canbus
}  // namespace apollo
