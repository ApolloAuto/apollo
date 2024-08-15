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
 * @file
 */

#pragma once

#include <memory>

#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "cyber/class_loader/class_loader_register_macro.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

using apollo::control::ControlCommand;

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class AbstractVehicleFactory
 *
 * @brief this class is the abstract factory following the AbstractFactory
 * design pattern. It can create VehicleController and MessageManager based on
 * a given VehicleParameter.
 */
class AbstractVehicleFactory {
 public:
  /**
   * @brief destructor
   */
  virtual ~AbstractVehicleFactory() = default;

  /**
   * @brief set VehicleParameter.
   */
  void SetVehicleParameter(const VehicleParameter &vehicle_paramter);

  /**
   * @brief init vehicle factory
   * @returns true if successfully initialized
   */
  virtual bool Init(const CanbusConf *canbus_conf) = 0;

  /**
   * @brief start canclient, cansender, canreceiver, vehicle controller
   * @returns true if successfully started
   */
  virtual bool Start() = 0;

  /**
   * @brief stop canclient, cansender, canreceiver, vehicle controller
   */
  virtual void Stop() = 0;

  /**
   * @brief update control command
   */
  virtual void UpdateCommand(const ControlCommand *control_command) = 0;

  /**
   * @brief update chassis command
   */
  virtual void UpdateCommand(const ChassisCommand *chassis_command) = 0;

  /**
   * @brief publish chassis messages
   */
  virtual Chassis publish_chassis() = 0;

  /**
   * @brief publish chassis for vehicle messages
   */
  virtual void PublishChassisDetail() = 0;

  /**
   * @brief publish chassis for vehicle messages
   */
  virtual void PublishChassisDetailSender();

  /**
   * @brief create cansender heartbeat
   */
  virtual void UpdateHeartbeat();

  /**
   * @brief check chassis detail communication fault
   */
  virtual bool CheckChassisCommunicationFault();

  /**
   * @brief add send protocol message
   */
  virtual void AddSendProtocol();

  /**
   * @brief clear send protocol message, make a sender can error
   */
  virtual void ClearSendProtocol();

  /**
   * @brief check send protocol message whether is clear or not
   */
  virtual bool IsSendProtocolClear();

  /**
   * @brief get chassis driving mode
   */
  virtual Chassis::DrivingMode Driving_Mode();

 private:
  VehicleParameter vehicle_parameter_;
};

#define CYBER_REGISTER_VEHICLEFACTORY(name) \
  CLASS_LOADER_REGISTER_CLASS(name, AbstractVehicleFactory)

}  // namespace canbus
}  // namespace apollo
