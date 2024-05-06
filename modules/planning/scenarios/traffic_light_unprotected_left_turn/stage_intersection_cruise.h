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
 **/

#pragma once

#include <string>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/traffic_light_base/base_stage_traffic_light_cruise.h"

namespace apollo {
namespace planning {

class TrafficLightUnprotectedLeftTurnStageIntersectionCruise
    : public BaseStageTrafficLightCruise {
 public:
  StageResult Process(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

 private:
  StageResult FinishStage();
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
    apollo::planning::TrafficLightUnprotectedLeftTurnStageIntersectionCruise,
    Stage)

}  // namespace planning
}  // namespace apollo
