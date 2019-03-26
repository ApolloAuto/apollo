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

#include "modules/planning/scenarios/util/util.h"

#include "modules/common/util/map_util.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace util {

using perception::TrafficLight;

/*
 * @brief: read signal info
 */
TrafficLight GetSignal(const std::string& traffic_light_id) {
  const auto* result = common::util::FindPtrOrNull(
      PlanningContext::GetScenarioInfo()->traffic_lights, traffic_light_id);

  if (result == nullptr) {
    TrafficLight traffic_light;
    traffic_light.set_id(traffic_light_id);
    traffic_light.set_color(TrafficLight::UNKNOWN);
    traffic_light.set_confidence(0.0);
    traffic_light.set_tracking_time(0.0);
    return traffic_light;
  }
  return *result;
}

}  // namespace util
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
