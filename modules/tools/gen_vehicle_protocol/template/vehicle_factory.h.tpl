/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
 * @file %(car_type_lower)s_vehicle_factory.h
 */

#pragma once

#include <memory>

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus_vehicle/%(car_type_lower)s/proto/%(car_type_lower)s.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

#include "cyber/cyber.h"
#include "modules/canbus/vehicle/abstract_vehicle_factory.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus_vehicle/%(car_type_lower)s/%(car_type_lower)s_controller.h"
#include "modules/canbus_vehicle/%(car_type_lower)s/%(car_type_lower)s_message_manager.h"
#include "modules/common/status/status.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class %(car_type_cap)sVehicleFactory
 *
 * @brief this class is inherited from AbstractVehicleFactory. It can be used to
 * create controller and message manager for %(car_type_lower)s vehicle.
 */
class %(car_type_cap)sVehicleFactory : public AbstractVehicleFactory {
 public:
  /**
   * @brief destructor
   */
  virtual ~%(car_type_cap)sVehicleFactory() = default;

  /**
   * @brief init vehicle factory
   * @returns true if successfully initialized
   */
  bool Init(const CanbusConf *canbus_conf) override;

  /**
   * @brief start canclient, cansender, canreceiver, vehicle controller
   * @returns true if successfully started
   */
  bool Start() override;

  /**
   * @brief stop canclient, cansender, canreceiver, vehicle controller
   */
  void Stop() override;

  /**
   * @brief update control command
   */
  void UpdateCommand(
      const apollo::control::ControlCommand *control_command) override;

  void UpdateCommand(
      const apollo::external_command::ChassisCommand *chassis_command) override;

  /**
   * @brief publish chassis messages
   */
  Chassis publish_chassis() override;

  /**
   * @brief create cansender heartbeat
   */
  void UpdateHeartbeat() override;

  /**
   * @brief publish chassis for vehicle messages
   */
  void PublishChassisDetail() override;

  /**
   * @brief publish chassis for apollo sender messages
   */
  void PublishChassisDetailSender() override;

  /**
   * @brief check chassis can receiver lost
   */
  bool CheckChassisCommunicationFault() override;

  /**
   * @brief add the can sender messages
   */
  void AddSendProtocol() override;

  /**
   * @brief clear the can sender messages
   */
  void ClearSendProtocol() override;

  /**
   * @brief check the sender message clear or not
   */
  bool IsSendProtocolClear() override;

  /**
   * @brief get the latest chassis driving mode
   */
  Chassis::DrivingMode Driving_Mode() override;

 private:
  /**
   * @brief create %(car_type_lower)s vehicle controller
   * @returns a unique_ptr that points to the created controller
   */
  std::unique_ptr<VehicleController<::apollo::canbus::%(car_type_cap)s>>
  CreateVehicleController();

  /**
   * @brief create %(car_type_lower)s message manager
   * @returns a unique_ptr that points to the created message manager
   */
  std::unique_ptr<MessageManager<::apollo::canbus::%(car_type_cap)s>> CreateMessageManager();

  std::unique_ptr<::apollo::cyber::Node> node_ = nullptr;
  std::unique_ptr<apollo::drivers::canbus::CanClient> can_client_;
  CanSender<::apollo::canbus::%(car_type_cap)s> can_sender_;
  apollo::drivers::canbus::CanReceiver<::apollo::canbus::%(car_type_cap)s> can_receiver_;
  std::unique_ptr<MessageManager<::apollo::canbus::%(car_type_cap)s>> message_manager_;
  std::unique_ptr<VehicleController<::apollo::canbus::%(car_type_cap)s>> vehicle_controller_;

  std::shared_ptr<::apollo::cyber::Writer<::apollo::canbus::%(car_type_cap)s>>
      chassis_detail_writer_;
  std::shared_ptr<::apollo::cyber::Writer<::apollo::canbus::%(car_type_cap)s>>
      chassis_detail_sender_writer_;
};

CYBER_REGISTER_VEHICLEFACTORY(%(car_type_cap)sVehicleFactory)

}  // namespace canbus
}  // namespace apollo
