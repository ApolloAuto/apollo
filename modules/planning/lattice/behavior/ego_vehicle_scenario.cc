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

#include "modules/planning/lattice/behavior/ego_vehicle_scenario.h"

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

void EgoVehicleScenario::Reset() {}

bool EgoVehicleScenario::Init() {
  exist_ = true;
  return exist_;
}

int EgoVehicleScenario::ComputeScenarioDecision(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    PlanningTarget* const decision) {
  CHECK(frame != nullptr);

  decision->set_cruise_speed(FLAGS_default_cruise_speed);

  return 0;
}

}  // namespace planning
}  // namespace apollo
