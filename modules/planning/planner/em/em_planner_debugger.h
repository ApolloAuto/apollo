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

#ifndef MODULES_PLANNING_PLANNER_EM_EM_PLANNER_DEBUGGER_H_
#define MODULES_PLANNING_PLANNER_EM_EM_PLANNER_DEBUGGER_H_

#include <unordered_map>
#include <string>

#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/reference_line/reference_point.h"

namespace apollo {
namespace planning {

class EMPlannerDebugger {
 public:
  std::unordered_map<std::string, std::pair<DiscretizedPath, double>> paths_;

  std::unordered_map<std::string, std::pair<std::vector<common::SpeedPoint>, double>> speed_profiles_;

  std::vector<ReferencePoint> reference_line_;
};

} //namespace planning
} //namespace apollo

#endif /* MODULES_PLANNING_PLANNER_EM_EM_PLANNER_DEBUGGER_H_ */
