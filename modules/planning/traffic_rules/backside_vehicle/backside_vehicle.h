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
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/planning/traffic_rules/backside_vehicle/proto/backside_vehicle.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

class BacksideVehicle : public TrafficRule {
 public:
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~BacksideVehicle() = default;
  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

  void Reset() override {}

 private:
  /**
   * @brief When the reference line info indicates that there is no lane change,
   * use lane keeping strategy for back side vehicles.
   */
  BacksideVehicleConfig config_;

  void MakeLaneKeepingObstacleDecision(
      const SLBoundary& adc_sl_boundary,
      PathDecision* path_decision,
      const common::VehicleState& vehicle_state);

  bool PredictionLineOverlapEgo(
      const Obstacle& obstacle, const common::VehicleState& vehicle_state);
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::BacksideVehicle,
                                     TrafficRule)

}  // namespace planning
}  // namespace apollo
