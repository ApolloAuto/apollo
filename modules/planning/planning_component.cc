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
#include "modules/planning/planning_component.h"

namespace apollo {
namespace planning {

using apollo::cybertron::Component;
using apollo::cybertron::ComponentBase;

bool PlanningComponent::Init() { return true; }

bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<perception::TrafficLightDetection>&
        traffic_light_detection,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  // implement here
  return true;
}

}  // namespace planning
}  // namespace apollo
