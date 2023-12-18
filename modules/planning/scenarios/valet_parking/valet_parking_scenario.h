/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/planning/scenarios/valet_parking/proto/valet_parking.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

struct ValetParkingContext : public ScenarioContext {
  ScenarioValetParkingConfig scenario_config;
  std::string target_parking_spot_id;
  bool pre_stop_rightaway_flag = false;
  hdmap::MapPathPoint pre_stop_rightaway_point;
};

class ValetParkingScenario : public Scenario {
 public:
  bool Init(std::shared_ptr<DependencyInjector> injector,
            const std::string& name) override;

  /**
   * @brief Get the scenario context.
   */
  ValetParkingContext* GetContext() override { return &context_; }

  bool IsTransferable(const Scenario* const other_scenario,
                      const Frame& frame) override;

 private:
  static bool SearchTargetParkingSpotOnPath(
      const hdmap::Path& nearby_path, const std::string& target_parking_id,
      hdmap::PathOverlap* parking_space_overlap);
  static bool CheckDistanceToParkingSpot(
      const Frame& frame, const common::VehicleState& vehicle_state,
      const hdmap::Path& nearby_path, const double parking_start_range,
      const hdmap::PathOverlap& parking_space_overlap);

 private:
  bool init_ = false;
  ValetParkingContext context_;
  const hdmap::HDMap* hdmap_ = nullptr;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ValetParkingScenario,
                                     Scenario)

}  // namespace planning
}  // namespace apollo
