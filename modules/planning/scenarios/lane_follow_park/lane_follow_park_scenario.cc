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

#include "modules/planning/scenarios/lane_follow_park/lane_follow_park_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/lane_follow_park/lane_follow_park_stage.h"

namespace apollo {
namespace planning {

bool LaneFollowParkScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioLaneFollowParkConfig>(&context_.scenario_config)) {
    AERROR << "failed to load config";
    return false;
  }

  return true;
}

bool LaneFollowParkScenario::IsTransferable(const Scenario* other_scenario,
                                        const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (frame.reference_line_info().empty()) {
    return false;
  }
  if (other_scenario == nullptr) {
    return true;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
