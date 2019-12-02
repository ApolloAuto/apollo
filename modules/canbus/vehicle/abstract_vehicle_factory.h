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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

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
   * @brief the interface of creating a VehicleController class
   * @returns a unique pointer that points to the created VehicleController
   * object.
   */
  virtual std::unique_ptr<VehicleController> CreateVehicleController() = 0;

  /**
   * @brief the interface of creating a MessageManager class
   * @returns a unique pointer that points to the created MessageManager object.
   */
  virtual std::unique_ptr<MessageManager<ChassisDetail>>
  CreateMessageManager() = 0;

  /**
   * @brief set VehicleParameter.
   */
  void SetVehicleParameter(const VehicleParameter &vehicle_paramter);

 private:
  VehicleParameter vehicle_parameter_;
};

}  // namespace canbus
}  // namespace apollo
