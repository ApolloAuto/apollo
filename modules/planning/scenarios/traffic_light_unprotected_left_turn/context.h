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
 * @file context.h
 */
#pragma once

#include <string>
#include <vector>

#include "modules/planning/scenarios/traffic_light_unprotected_left_turn/proto/traffic_light_unprotected_left_turn.pb.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

// stage context
struct TrafficLightUnprotectedLeftTurnContext : public ScenarioContext {
  ScenarioTrafficLightUnprotectedLeftTurnConfig scenario_config;
  std::vector<std::string> current_traffic_light_overlap_ids;
  double creep_start_time;
};

}  // namespace planning
}  // namespace apollo
