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

#include "modules/planning/planner_factory.h"

#include "modules/planning/planner/rtk_replay_planner.h"

namespace apollo {
namespace planning {

std::unique_ptr<Planner> PlannerFactory::CreateInstance(
    const PlannerType& planner_type) {
  switch (planner_type) {
    case PlannerType::RTK_PLANNER:
      return std::unique_ptr<Planner>(new RTKReplayPlanner());
    default:
      return nullptr;
  }
}

}  // namespace planning
}  // namespace apollo
