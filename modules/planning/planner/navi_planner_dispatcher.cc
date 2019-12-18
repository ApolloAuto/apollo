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

#include "modules/planning/planner/navi_planner_dispatcher.h"

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

std::unique_ptr<Planner> NaviPlannerDispatcher::DispatchPlanner() {
  PlanningConfig planning_config;
  if (!apollo::cyber::common::GetProtoFromFile(FLAGS_planning_config_file,
                                               &planning_config)) {
    return nullptr;
  }

  auto planner_type = PlannerType::NAVI;
  if (planning_config.has_navigation_planning_config()) {
    planner_type = planning_config.navigation_planning_config().planner_type(0);
  }
  return planner_factory_.CreateObject(planner_type);
}

}  // namespace planning
}  // namespace apollo
