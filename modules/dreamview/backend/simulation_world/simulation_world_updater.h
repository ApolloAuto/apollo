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

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <string>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"

#include "modules/dreamview/backend/data_collection_monitor/data_collection_monitor.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/perception_camera_updater/perception_camera_updater.h"
#include "modules/dreamview/backend/sim_control/sim_control.h"
#include "modules/dreamview/backend/simulation_world/simulation_world_service.h"
#include "modules/routing/proto/poi.pb.h"

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
   * @param sim_control Pointer of sim control.
   * @param map_service Pointer of the map service to provide a high-level API
   * of hdmap.
   * @param routing_from_file whether to read initial routing from file.
   */
  SimulationWorldUpdater(WebSocketHandler *websocket, WebSocketHandler *map_ws,
                         WebSocketHandler *camera_ws, SimControl *sim_control,
                         const MapService *map_service,
                         DataCollectionMonitor *data_collection_monitor,
                         PerceptionCameraUpdater *perception_camera_updater,
                         bool routing_from_file = false);

  /**
   * @brief Starts to push simulation_world to frontend.
   */
  void Start();

  // Time interval, in milliseconds, between pushing SimulationWorld to
  // frontend.
  static constexpr double kSimWorldTimeIntervalMs = 100;

 private:
  /**
   * @brief The callback function to get updates from SimulationWorldService,
   * and update simulation_world_json_.
   */
  void OnTimer();

  /**
   * @brief The function to construct a routing request from the given json,
   * @param json that contains start, end, and waypoints
   * @param routing_request
   * @return True if routing request is constructed successfully
   */
  bool ConstructRoutingRequest(
      const nlohmann::json &json,
      apollo::routing::RoutingRequest *routing_request);

  bool ValidateCoordinate(const nlohmann::json &json);

  /**
   * @brief Check if routing point is located on a lane that is CITY_DRIVING
   * @param json that contains point's coordinate x and y
   * @return True if the lane is CITY_DRIVING
   */
  nlohmann::json CheckRoutingPoint(const nlohmann::json &json);

  /**
   * @brief Tries to load the points of interest from the file if it has
   * not been.
   * @return False if failed to load from file,
   * true otherwise or if it's already loaded.
   */
  bool LoadPOI();

  void RegisterMessageHandlers();

  std::unique_ptr<cyber::Timer> timer_;

  SimulationWorldService sim_world_service_;
  const MapService *map_service_ = nullptr;
  WebSocketHandler *websocket_ = nullptr;
  WebSocketHandler *map_ws_ = nullptr;
  WebSocketHandler *camera_ws_ = nullptr;
  SimControl *sim_control_ = nullptr;
  DataCollectionMonitor *data_collection_monitor_ = nullptr;
  PerceptionCameraUpdater *perception_camera_updater_ = nullptr;

  // End point for requesting default route
  apollo::routing::POI poi_;

  // The simulation_world in wire format to be pushed to frontend, which is
  // updated by timer.
  std::string simulation_world_;
  std::string simulation_world_with_planning_data_;

  // Received relative map data in wire format.
  std::string relative_map_string_;

  // Mutex to protect concurrent access to simulation_world_json_.
  // NOTE: Use boost until we have std version of rwlock support.
  boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo
