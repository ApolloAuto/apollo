/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_HMI_TOPICS_UPDATER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_HMI_TOPICS_UPDATER_H_

#include <memory>
#include <string>
#include <vector>

#include "boost/thread/locks.hpp"
#include "boost/thread/shared_mutex.hpp"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/tools/navi_generator/backend/hmi/topics_service.h"
#include "modules/tools/navi_generator/backend/webserver/navi_generator_websocket.h"

namespace apollo {
namespace navi_generator {

class TopicsUpdater {
 public:
  /**
   * @brief Constructor with the websocket handler.
   * @param websocket Pointer of the websocket handler that has been attached to
   * the server.
   */
  explicit TopicsUpdater(NaviGeneratorWebSocket *websocket);

  /**
   * @brief Starts to push simulation_world to frontend.
   */
  void Start();

 private:
  /**
   * @brief The callback function to get updates from SimulationWorldService,
   * and update simulation_world_json_.
   * @param event Timer event
   */
  void OnTimer(const ros::TimerEvent &event);

  void RegisterMessageHandlers();
  bool ValidateCoordinate(const nlohmann::json &json);

  // Broadcast SystemStatus to all clients.
  void StartBroadcastHMIStatusThread();
  void DeferredBroadcastHMIStatus();

 private:
  NaviGeneratorWebSocket *websocket_ = nullptr;

  ros::Timer timer_;
  TopicsService topicsService_;

  // For HMIStatus broadcasting.
  std::unique_ptr<std::thread> broadcast_hmi_status_thread_;
  bool need_broadcast_ = false;
  std::mutex need_broadcast_mutex_;
};

}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_HMI_TOPICS_UPDATER_H_
