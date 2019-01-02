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

#include "modules/planning/tasks/optimizers/proceed_with_caution_speed/proceed_with_caution_speed_generator.h"

#include <algorithm>
#include <string>

#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

ProceedWithCautionSpeedGenerator::ProceedWithCautionSpeedGenerator(
    const TaskConfig& config)
    : SpeedOptimizer(config) {
  CHECK(config_.has_proceed_with_caution_speed_config());
  SetName("ProceedWithCautionSpeedGenerator");
}

Status ProceedWithCautionSpeedGenerator::Process(
    const SLBoundary& adc_sl_boundary, const PathData& path_data,
    const TrajectoryPoint& init_point, const ReferenceLine& reference_line,
    const SpeedData& reference_speed_data, PathDecision* const path_decision,
    SpeedData* const speed_data) {
  if (path_data.discretized_path().empty()) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  speed_data->clear();

  auto proceed_param =
      PlanningContext::GetScenarioInfo()->proceed_with_caution_speed;
  const bool is_fixed_distance = proceed_param.is_fixed_distance;
  double proceed_distance = is_fixed_distance
                                ? proceed_param.distance
                                : path_data.discretized_path().Length();
  proceed_distance =
      std::min(proceed_distance,
               config_.proceed_with_caution_speed_config().max_distance());

  *speed_data = SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(
      proceed_distance, proceeding_speed_);

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
