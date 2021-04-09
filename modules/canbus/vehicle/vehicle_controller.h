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

#ifndef MODULES_CANBUS_VEHICLE_CONTROLLER_H_
#define MODULES_CANBUS_VEHICLE_CONTROLLER_H_

#include <unordered_map>

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/error_code.pb.h"
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
      const VehicleParameter &params,
      CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
      MessageManager<::apollo::canbus::ChassisDetail>
          *const message_manager) = 0;

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

 protected:
  virtual Chassis::DrivingMode driving_mode();
  virtual void set_driving_mode(const Chassis::DrivingMode &driving_mode);

 protected:
  canbus::VehicleParameter params_;
  common::VehicleParam vehicle_params_;
  CanSender<::apollo::canbus::ChassisDetail> *can_sender_ = nullptr;
  MessageManager<::apollo::canbus::ChassisDetail> *message_manager_ = nullptr;
  bool is_initialized_ = false;  // own by derviative concrete controller
  Chassis::DrivingMode driving_mode_ = Chassis::COMPLETE_MANUAL;
  bool is_reset_ = false;  // reset command from control command
  std::mutex mode_mutex_;  // only use in this base class
};

}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_CONTROLLER_H_
