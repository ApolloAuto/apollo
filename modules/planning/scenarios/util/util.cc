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

#include <vector>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {
namespace scenario {

using hdmap::PathOverlap;
using perception::TrafficLight;

bool CheckStopSignDone(const ReferenceLineInfo& reference_line_info,
                       const std::string& stop_sign_overlap_id) {
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_it =
      std::find_if(stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
                   [&stop_sign_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == stop_sign_overlap_id;
                   });
  return (stop_sign_overlap_it == stop_sign_overlaps.end());
}

bool CheckTrafficLightDone(const ReferenceLineInfo& reference_line_info,
                           const std::string& traffic_light_overlap_id) {
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  return (traffic_light_overlap_it == traffic_light_overlaps.end());
}

TrafficLight GetSignal(const std::string& traffic_light_id) {
  const auto* result = apollo::common::util::FindPtrOrNull(
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

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
