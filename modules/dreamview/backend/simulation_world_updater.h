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

#ifndef MODULES_DREAMVIEW_BACKEND_SIM_WORLD_UPDATER_H_
#define MODULES_DREAMVIEW_BACKEND_SIM_WORLD_UPDATER_H_

#include "modules/common/log.h"
#include "modules/dreamview/backend/map_service.h"
#include "modules/dreamview/backend/simulation_world_service.h"
#include "modules/dreamview/backend/websocket.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class SimulationWorldUpdater
 * @brief A wrapper around SimulationWorldService and WebSocketHandler to keep
 * pushing SimulationWorld to frontend via websocket while handling the response
 * from frontend.
 */
class SimulationWorldUpdater {
 public:
  /**
   * @brief Constructor with the websocket handler.
   * @param websocket Pointer of the websocket handler that has been attached to
   * the server.
   */
  explicit SimulationWorldUpdater(WebSocketHandler *websocket);

  /**
   * @brief The callback function to get updates from SimulationWorldService,
   * and push them to the frontend clients via websocket when the periodic timer
   * is triggered.
   * @param event Timer event
   */
  void OnPushTimer(const ros::TimerEvent &event);

 private:
  MapService map_service_;
  SimulationWorldService sim_world_service_;
  WebSocketHandler *websocket_;
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_SIM_WORLD_UPDATER_H_
