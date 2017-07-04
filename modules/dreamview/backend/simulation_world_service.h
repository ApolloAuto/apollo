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

#ifndef MODULES_DREAMVIEW_BACKEND_SIM_WORLD_H_
#define MODULES_DREAMVIEW_BACKEND_SIM_WORLD_H_

#include <functional>
#include <string>
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/dreamview/proto/simulation_world.pb.h"
#include "third_party/json/json.hpp"

/**
 * @namespace apollo::dreamview::internal
 * @brief apollo::dreamview::internal
 */
namespace apollo {
namespace dreamview {

namespace internal {

template <typename AdapterType>
void UpdateSimulationWorld(const typename AdapterType::DataType &data,
                           SimulationWorld *world);

template <>
void UpdateSimulationWorld<apollo::common::adapter::MonitorAdapter>(
    const apollo::common::monitor::MonitorMessage &monitor_msg,
    SimulationWorld *world);

template <>
void UpdateSimulationWorld<apollo::common::adapter::LocalizationAdapter>(
    const apollo::localization::LocalizationEstimate &localization,
    SimulationWorld *world);

template <>
void UpdateSimulationWorld<apollo::common::adapter::ChassisAdapter>(
    const apollo::canbus::Chassis &chassis, SimulationWorld *world);

template <>
void UpdateSimulationWorld<apollo::common::adapter::PlanningTrajectoryAdapter>(
    const apollo::planning::ADCTrajectory &trajectory, SimulationWorld *world);

}  // namespace internal

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
  using Json = nlohmann::json;

  // The maximum number of monitor message items to be kept in
  // SimulationWorld.
  static constexpr int kMaxMonitorItems = 30;

  /**
   * @brief Default constructor.
   */
  SimulationWorldService();

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
  Json GetUpdateAsJson() const;

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
    return world_.has_auto_driving_car();
  }

 private:
  /**
   * @brief Register a callback on the adatper manager to update the
   * SimulationWorld object upon receiving a new message. This is not
   * guarded by lock since we are using single threaded ROS spinner.
   */
  template <typename AdapterType>
  void RegisterDataCallback(const std::string &adapter_name,
                            AdapterType *adapter) {
    if (adapter == nullptr) {
      AFATAL << adapter_name << " adapter is not correctly initialized. "
                                "Please check the adapter manager ";
    }

    adapter->SetCallback(
        std::bind(&internal::UpdateSimulationWorld<AdapterType>,
                  std::placeholders::_1, &world_));
  }

  // The underlying SimulationWorld object, owned by the
  // SimulationWorldService instance.
  SimulationWorld world_;
};

}  // namespace dreamview
}  // namespace apollo

#endif /* MODULES_DREAMVIEW_BACKEND_SIM_WORLD_H_ */
