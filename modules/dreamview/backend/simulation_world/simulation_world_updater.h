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

#include <string>

#include "boost/thread/locks.hpp"
#include "boost/thread/shared_mutex.hpp"

#include "modules/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/dreamview/backend/handlers/websocket.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/sim_control/sim_control.h"
#include "modules/dreamview/backend/simulation_world/simulation_world_service.h"

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
  SimulationWorldUpdater(WebSocketHandler *websocket, SimControl *sim_control,
                         const MapService *map_service,
                         bool routing_from_file = false);

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

  /**
   * @brief The function to construct a routing request from the given json,
   * @param json that contains start, end, and waypoints
   * @param routing_request
   * @return True if routing request is constructed successfully
   */
  bool ConstructRoutingRequest(
      const nlohmann::json &json,
      apollo::routing::RoutingRequest *routing_request);

  /**
   * @brief Tries to load the default routing end point from the file if it has
   * not been.
   * @return False if failed to load the default routing end point from file,
   * true otherwise or if it's already loaded.
   */
  bool LoadDefaultEndPoint();

  /**
   * @brief Dumps the latest received message to file.
   * @param adapter the adapter to perfom dumping
   * @param adapter_name the name of the adapter
   */
  template <typename AdapterType>
  void DumpMessage(AdapterType *adapter, std::string adapter_name) {
    if (adapter->DumpLatestMessage()) {
      sim_world_service_.PublishMonitorMessage(
          common::monitor::MonitorMessageItem::INFO,
          common::util::StrCat("Dumped latest ", adapter_name,
                               " message under /tmp/", adapter_name, "."));
    } else {
      sim_world_service_.PublishMonitorMessage(
          common::monitor::MonitorMessageItem::WARN,
          common::util::StrCat("Failed to dump latest ", adapter_name,
                               " message."));
    }
  }

  // Time interval, in seconds, between pushing SimulationWorld to frontend.
  static constexpr double kSimWorldTimeInterval = 0.1;

  ros::Timer timer_;
  SimulationWorldService sim_world_service_;
  const MapService *map_service_;
  WebSocketHandler *websocket_;
  SimControl *sim_control_;

  // End point for requesting default route
  apollo::routing::LaneWaypoint default_end_point_;

  // The json string to be pushed to frontend, which is updated by timer.
  std::string simulation_world_json_;

  // Mutex to protect concurrent access to simulation_world_json_.
  // NOTE: Use boost until we have std version of rwlock support.
  boost::shared_mutex mutex_;
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_SIMULATION_WORLD_SIM_WORLD_UPDATER_H_
