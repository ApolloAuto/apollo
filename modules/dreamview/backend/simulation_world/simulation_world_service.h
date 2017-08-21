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

#include "gtest/gtest_prod.h"
#include "third_party/json/json.hpp"

#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/proto/simulation_world.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"

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

  // The radius within which Dreamview will find all the map elements around the
  // car.
  static constexpr double kMapRadius = 200.0;

  /**
   * @brief Constructor of SimulationWorldService.
   * @param map_service the pointer of MapService.
   * @param routing_from_file whether to read intial routing from file.
   */
  SimulationWorldService(MapService *map_service,
                         bool routing_from_file = false);

  /**
   * @brief Get a read-only view of the SimulationWorld.
   * @return Constant reference to the SimulationWorld object.
   */
  inline const SimulationWorld &world() const {
    return world_;
  }

  /**
   * @brief Returns the json representation of the SimulationWorld object.
   * @return Json object equivalence of the SimulationWorld object.
   */
  nlohmann::json GetUpdateAsJson() const;

  /**
   * @brief The function Update() is periodically called to check for updates
   * from the adapters. All the updates will be written to the SimulationWorld
   * object to reflect the latest status.
   * @return Constant reference to the SimulationWorld object.
   */
  const SimulationWorld &Update();

  /**
   * @brief Check whether the SimulationWorld object has enough information.
   * The backend won't push the SimulationWorld to frontend if it is not ready.
   * @return True if the object is ready to push.
   */
  bool ReadyToPush() const {
    return world_.has_auto_driving_car() &&
           world_.auto_driving_car().has_position_x() &&
           world_.auto_driving_car().has_position_y();
  }

 private:
  /**
   * @brief Update simulation world with incoming data, e.g., chassis,
   * localization, planning, perception, etc.
   */
  template <typename DataType>
  void UpdateSimulationWorld(const DataType &data);

  Object &CreateWorldObjectIfAbsent(
      const apollo::perception::PerceptionObstacle &obstacle);
  void UpdatePlanningTrajectory(
      const apollo::planning::ADCTrajectory &trajectory);
  void UpdateDecision(const apollo::planning::DecisionResult &decision_res,
                      double header_time);
  void UpdateMainDecision(const apollo::planning::MainDecision &main_decision,
                          double update_timestamp_sec, Object *world_main_stop);

  /**
   * @brief Check whether a particular adapter has been initialized correctly.
   */
  template <typename AdapterType>
  bool CheckAdapterInitialized(const std::string &adapter_name,
                               AdapterType *adapter) {
    if (adapter == nullptr) {
      AERROR << adapter_name << " adapter is not correctly initialized. "
                                "Please check the adapter manager "
                                "configuration.";
      return false;
    }
    return true;
  }

  /**
   * @brief Get the latest observed data from the adatper manager to update the
   * SimulationWorld object when triggered by refresh timer.
   */
  template <typename AdapterType>
  void UpdateWithLatestObserved(const std::string &adapter_name,
                                AdapterType *adapter) {
    if (!CheckAdapterInitialized(adapter_name, adapter)) {
      return;
    }

    if (adapter->Empty()) {
      AINFO << adapter_name << " adapter has not received any data yet.";
      return;
    }

    UpdateSimulationWorld(adapter->GetLatestObserved());
  }

  void RegisterMessageCallbacks();

  void ReadRoutingFromFile(const std::string &routing_response_file);

  // The underlying SimulationWorld object, owned by the
  // SimulationWorldService instance.
  SimulationWorld world_;

  // The handle of MapService, not owned by SimulationWorldService.
  MapService *map_service_;

  // The map holding obstacle string id to the actual object
  std::unordered_map<std::string, Object> obj_map_;

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
