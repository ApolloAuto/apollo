/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file zhongyun_vehicle_factory.h
 */

#pragma once

#include <memory>

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus_vehicle/zhongyun/proto/zhongyun.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "cyber/cyber.h"
#include "modules/canbus/vehicle/abstract_vehicle_factory.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
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
 * @class ZhongyunVehicleFactory
 *
 * @brief this class is inherited from AbstractVehicleFactory. It can be used to
 * create controller and message manager for zhongyun vehicle.
 */
class ZhongyunVehicleFactory : public AbstractVehicleFactory {
 public:
  /**
   * @brief destructor
   */
  virtual ~ZhongyunVehicleFactory() = default;

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
   * @brief create ch vehicle controller
   * @returns a unique_ptr that points to the created controller
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
   * @brief publish chassis for vehicle messages
   */
  void PublishChassisDetail() override;

 private:
  /**
   * @brief create zhongyun vehicle controller
   * @returns a unique_ptr that points to the created controller
   */
  std::unique_ptr<VehicleController<::apollo::canbus::Zhongyun>>
  CreateVehicleController();

  /**
   * @brief create zhongyun message manager
   * @returns a unique_ptr that points to the created message manager
   */
  std::unique_ptr<MessageManager<::apollo::canbus::Zhongyun>>
  CreateMessageManager();

  std::unique_ptr<::apollo::cyber::Node> node_ = nullptr;
  std::unique_ptr<apollo::drivers::canbus::CanClient> can_client_;
  CanSender<::apollo::canbus::Zhongyun> can_sender_;
  apollo::drivers::canbus::CanReceiver<::apollo::canbus::Zhongyun>
      can_receiver_;
  std::unique_ptr<MessageManager<::apollo::canbus::Zhongyun>> message_manager_;
  std::unique_ptr<VehicleController<::apollo::canbus::Zhongyun>>
      vehicle_controller_;

  std::shared_ptr<::apollo::cyber::Writer<::apollo::canbus::Zhongyun>>
      chassis_detail_writer_;
};

CYBER_REGISTER_VEHICLEFACTORY(ZhongyunVehicleFactory)

}  // namespace canbus
}  // namespace apollo
