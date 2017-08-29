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

#ifndef MODULES_DREAMVIEW_BACKEND_SIMULATION_WORLD_SIM_WORLD_UPDATER_H_
#define MODULES_DREAMVIEW_BACKEND_SIMULATION_WORLD_SIM_WORLD_UPDATER_H_

#include "modules/common/log.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/simulation_world/simulation_world_service.h"
#include "modules/dreamview/backend/websocket/websocket.h"

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
   * @param map_service Pointer of the map service to provide a high-level API
   * of hdmap.
   * @param routing_from_file whether to read intial routing from file.
   */
  SimulationWorldUpdater(WebSocketHandler *websocket,
                         const MapService *map_service,
                         bool routing_from_file = false);

  /**
   * @brief Starts to push simulation_world to frontend.
   */
  void Start();

 private:
  /**
   * @brief The callback function to get updates from SimulationWorldService,
   * and push them to the frontend clients via websocket when the periodic timer
   * is triggered.
   * @param event Timer event
   */
  void OnPushTimer(const ros::TimerEvent &event);

  /**
   * @brief The function to construct a routing request from the given json,
   * @param json that contains start, end, and waypoints
   * @param routing_request
   * @return True if routing request is constructed successfully
   */
  bool ConstructRoutingRequest(const nlohmann::json &json,
                               routing::RoutingRequest* routing_request);

  // Time interval, in seconds, between pushing SimulationWorld to frontend.
  static constexpr double kSimWorldTimeInterval = 0.1;

  ros::Timer timer_;
  SimulationWorldService sim_world_service_;
  const MapService *map_service_;
  WebSocketHandler *websocket_;
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_SIMULATION_WORLD_SIM_WORLD_UPDATER_H_
