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

#ifndef MODULES_DREAMVIEW_BACKEND_HMI_HMI_H_
#define MODULES_DREAMVIEW_BACKEND_HMI_HMI_H_

#include <string>

#include "gtest/gtest_prod.h"
#include "modules/dreamview/backend/handlers/websocket.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/proto/hmi_config.pb.h"
#include "modules/dreamview/proto/hmi_status.pb.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

class HMI {
 public:
  HMI(WebSocketHandler *websocket, MapService *map_service);

 private:
  // Broadcast HMIStatus to all clients.
  void BroadcastHMIStatus() const;

  void RegisterMessageHandlers();

  // Run a supported command of some component, return the ret code.
  static int RunComponentCommand(
      const google::protobuf::Map<std::string, Component> &components,
      const std::string &component_name, const std::string &command_name);

  // Run a supported command of current mode.
  void RunModeCommand(const std::string &command_name);

  static void ChangeDrivingModeTo(const std::string &new_mode);
  void ChangeMapTo(const std::string &map_name);
  void ChangeVehicleTo(const std::string &vehicle_name);
  void ChangeModeTo(const std::string &mode_name);

  // Check if there is available updates.
  void CheckOTAUpdates();

  HMIConfig config_;
  HMIStatus status_;

  // No ownership.
  WebSocketHandler *websocket_;
  MapService *map_service_;

  FRIEND_TEST(HMITest, RunComponentCommand);
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_HMI_HMI_H_
