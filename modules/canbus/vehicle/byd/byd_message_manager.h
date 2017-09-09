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
 * @file byd_message_manager.h
 * @brief the class of BydMessageManager
 */

 #ifndef MODULES_CANBUS_VEHICLE_BYD_MESSAGE_MANAGER_H_
 #define MODULES_CANBUS_VEHICLE_BYD_MESSAGE_MANAGER_H_

 #include "modules/canbus/vehicle/message_manager.h"
/**
* @namespace apollo::canbus::lincoln
* @brief apollo::canbus::lincoln
*/
namespace apollo {
namespace canbus {
namespace byd {

/**
* @class BydMessageManager
*
* @brief implementation of MessageManager for byd vehicle
*/
class BydMessageManager : public MessageManager {
public:
/**
* @brief construct a byd message manager. protocol data for send and
* receive are added in the contruction.
*/
  BydMessageManager();
  virtual ~BydMessageManager();
};

}  // namespace byd
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_BYD_MESSAGE_MANAGER_H_
