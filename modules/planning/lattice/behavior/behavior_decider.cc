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

#include "modules/planning/lattice/behavior/behavior_decider.h"

#include <string>

#include "gflags/gflags.h"
#include "modules/planning/lattice/behavior/scenario_manager.h"
#include "modules/common/log.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

PlanningTarget BehaviorDecider::Analyze(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    const TrajectoryPoint& init_planning_point,
    const std::array<double, 3>& lon_init_state,
    const std::vector<PathPoint>& discretized_reference_line) {
  CHECK(frame != nullptr);
  CHECK_GT(discretized_reference_line.size(), 0);

  PlanningTarget planning_target;
  if (ScenarioManager::instance()->ComputeWorldDecision(
          frame, reference_line_info, &planning_target) != 0) {
    AERROR << "ComputeWorldDecision error!";
  }

  CHECK(FLAGS_default_cruise_speed <= FLAGS_planning_upper_speed_limit);

  return planning_target;
}

}  // namespace planning
}  // namespace apollo
