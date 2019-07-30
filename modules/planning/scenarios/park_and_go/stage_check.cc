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

#include "modules/planning/scenarios/park_and_go/stage_check.h"

#include <string>
#include <vector>

#include "cyber/common/log.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace park_and_go {

using common::TrajectoryPoint;

Stage::StageStatus ParkAndGoStageCheck::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Check";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  bool success = CheckObstacle(reference_line_info);
  return FinishStage(success);
}

// TODO(SHU): reverse_driving;
// check front obstacle:
// a. no obstacle;
// b. obstacle is half vehicle length away
bool ParkAndGoStageCheck::CheckObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const double kClearance = 2.0;
  const auto& reference_line = reference_line_info.reference_line();
  const auto& path_decision = reference_line_info.path_decision();
  const auto& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (reference_line.IsOnLane(obstacle->PerceptionSLBoundary()) &&
        obstacle->PerceptionSLBoundary().start_s() <
            adc_sl_boundary.end_s() + kClearance) {
      return false;
    }
  }
  return true;
}

Stage::StageStatus ParkAndGoStageCheck::FinishStage(const bool success) {
  if (success) {
    next_stage_ = ScenarioConfig::PARK_AND_GO_CRUISE;
  } else {
    next_stage_ = ScenarioConfig::PARK_AND_GO_ADJUST;
  }
  return Stage::FINISHED;
}

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
