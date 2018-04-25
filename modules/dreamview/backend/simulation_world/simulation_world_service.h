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

#ifndef MODULES_DREAMVIEW_BACKEND_SIMULATION_WORLD_SIM_WORLD_H_
#define MODULES_DREAMVIEW_BACKEND_SIMULATION_WORLD_SIM_WORLD_H_

#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "third_party/json/json.hpp"

#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/proto/simulation_world.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/proto/pnc_point.pb.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class SimulationWorldService
 * @brief This is a major component of the Simulation backend, which
 * maintains a SimulationWorld object and keeps updating it. The SimulationWorld
 * represents the most up-to-date information about all the objects
 * in the emulated world, including the car, the planning trajectory, etc.
 * NOTE: This class is not thread-safe.
 */
class SimulationWorldService {
 public:
  // The maximum number of monitor message items to be kept in
  // SimulationWorld.
  static constexpr int kMaxMonitorItems = 30;

  /**
   * @brief Constructor of SimulationWorldService.
   * @param map_service the pointer of MapService.
   * @param routing_from_file whether to read initial routing from file.
   */
  SimulationWorldService(const MapService *map_service,
                         bool routing_from_file = false);

  /**
   * @brief Get a read-only view of the SimulationWorld.
   * @return Constant reference to the SimulationWorld object.
   */
  inline const SimulationWorld &world() const { return world_; }

  /**
   * @brief Returns the json representation of the SimulationWorld object.
   *        This is a public API used by offline dreamview.
   * @param radius the search distance from the current car location
   * @return Json object equivalence of the SimulationWorld object.
   */
  nlohmann::json GetUpdateAsJson(double radius) const;

  /**
   * @brief Returns the binary representation of the SimulationWorld object.
   * @param radius the search distance from the current car location.
   * @param sim_world output of binary format sim_world string.
   * @param sim_world_with_planning_data output of binary format sim_world
   * string with planning_data.
   */
  void GetWireFormatString(double radius, std::string *sim_world,
                           std::string *sim_world_with_planning_data);

  /**
   * @brief Returns the json representation of the map element Ids and hash
   * within the given radius from the car.
   * @param radius the search distance from the current car location
   * @return Json object that contains mapElementIds and mapHash.
   */
  nlohmann::json GetMapElements(double radius) const;

  /**
   * @brief The function Update() is periodically called to check for updates
   * from the adapters. All the updates will be written to the SimulationWorld
   * object to reflect the latest status.
   */
  void Update();

  /**
   * @brief Sets the flag to clear the owned simulation world object.
   */
  void SetToClear() { to_clear_ = true; }

  /**
   * @brief Check whether the SimulationWorld object has enough information.
   * The backend won't push the SimulationWorld to frontend if it is not ready.
   * @return True if the object is ready to push.
   */
  bool ReadyToPush() const { return ready_to_push_.load(); }

  /**
   * @brief Publish message to the monitor
   * @param msg the string to send to monitor
   * @param log_level defined in
   *        modules/common/monitor_log/proto/monitor_log.proto
   */
  void PublishMonitorMessage(
      apollo::common::monitor::MonitorMessageItem::LogLevel log_level,
      const std::string &msg) {
    apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.AddMonitorMsgItem(log_level, msg);
  }

  void GetMapElementIds(double radius, MapElementIds *ids) const;

  const apollo::hdmap::Map &GetRelativeMap() const;

  nlohmann::json GetRoutePathAsJson() const;

 private:
  /**
   * @brief Update simulation world with incoming data, e.g., chassis,
   * localization, planning, perception, etc.
   */
  template <typename DataType>
  void UpdateSimulationWorld(const DataType &data);

  Object &CreateWorldObjectIfAbsent(
      const apollo::perception::PerceptionObstacle &obstacle);
  void SetObstacleInfo(const apollo::perception::PerceptionObstacle &obstacle,
                       Object *world_object);
  void SetObstaclePolygon(
      const apollo::perception::PerceptionObstacle &obstacle,
      Object *world_object);
  void UpdatePlanningTrajectory(
      const apollo::planning::ADCTrajectory &trajectory);
  bool LocateMarker(const apollo::planning::ObjectDecisionType &decision,
                    Decision *world_decision);
  void FindNudgeRegion(const apollo::planning::ObjectDecisionType &decision,
                       const Object &world_obj, Decision *world_decision);
  void UpdateDecision(const apollo::planning::DecisionResult &decision_res,
                      double header_time);
  void UpdateMainStopDecision(
      const apollo::planning::MainDecision &main_decision,
      double update_timestamp_sec, Object *world_main_stop);
  template <typename MainDecision>
  void UpdateMainChangeLaneDecision(const MainDecision &decision,
                                    Object *world_main_decision);

  void CreatePredictionTrajectory(
      const apollo::prediction::PredictionObstacle &obstacle,
      Object *world_object);

  void DownsamplePath(const apollo::common::Path &paths,
                      apollo::common::Path *downsampled_path);

  void UpdatePlanningData(const apollo::planning_internal::PlanningData &data);

  void PopulateMapInfo(double radius);

  /**
   * @brief Get the latest observed data from the adapter manager to update the
   * SimulationWorld object when triggered by refresh timer.
   */
  template <typename AdapterType>
  void UpdateWithLatestObserved(const std::string &adapter_name,
                                AdapterType *adapter, bool logging = true) {
    if (adapter->Empty()) {
      if (logging) {
        AINFO_EVERY(100) << adapter_name
                         << " adapter has not received any data yet.";
      }
      return;
    }

    UpdateSimulationWorld(adapter->GetLatestObserved());
  }

  void RegisterMessageCallbacks();

  void ReadRoutingFromFile(const std::string &routing_response_file);

  void UpdateDelays();

  template <typename Points>
  void DownsampleSpeedPointsByInterval(const Points &points,
                                       size_t downsampleInterval,
                                       Points *downsampled_points);

  // The underlying SimulationWorld object, owned by the
  // SimulationWorldService instance.
  SimulationWorld world_;

  // Downsampled route paths to be rendered in frontend.
  std::vector<RoutePath> route_paths_;

  // The handle of MapService, not owned by SimulationWorldService.
  const MapService *map_service_;

  // The map holding obstacle string id to the actual object
  std::unordered_map<std::string, Object> obj_map_;

  // The SIMULATOR monitor for publishing messages.
  apollo::common::monitor::MonitorLogger monitor_logger_;

  // Whether to clear the SimulationWorld in the next timer cycle, set by
  // frontend request.
  bool to_clear_ = false;

  // Relative map used/retrieved in navigation mode
  apollo::hdmap::Map relative_map_;

  // Whether the sim_world is ready to push to frontend
  std::atomic<bool> ready_to_push_;

  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorSuccess);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorRemove);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateMonitorTruncate);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateChassisInfo);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateLocalization);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePerceptionObstacles);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePlanningTrajectory);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateDecision);
  FRIEND_TEST(SimulationWorldServiceTest, UpdatePrediction);
  FRIEND_TEST(SimulationWorldServiceTest, UpdateRouting);
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_SIMULATION_WORLD_SIM_WORLD_H_
