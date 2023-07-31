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
 * @file lincoln_message_manager.h
 * @brief the class of LincolnMessageManager
 */

#pragma once

#include "modules/canbus_vehicle/lincoln/proto/lincoln.pb.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"

/**
 * @namespace apollo::canbus::lincoln
 * @brief apollo::canbus::lincoln
 */
namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::MessageManager;

/**
 * @class LincolnMessageManager
 *
 * @brief implementation of MessageManager for lincoln vehicle
 */
class LincolnMessageManager : public MessageManager<::apollo::canbus::Lincoln> {
 public:
  /**
   * @brief construct a lincoln message manager. protocol data for send and
   * receive are added in the construction.
   */
  LincolnMessageManager();
  virtual ~LincolnMessageManager();
};

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
