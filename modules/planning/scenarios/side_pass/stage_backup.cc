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

#include "modules/planning/scenarios/side_pass/stage_backup.h"

#include <algorithm>
#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleConfigHelper;

/*
 * @brief:
 * STAGE: StageBackup
 */
Stage::StageStatus StageBackup::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  // check the status of side pass scenario
  const SLBoundary& adc_sl_boundary =
      frame->reference_line_info().front().AdcSlBoundary();
  const PathDecision& path_decision =
      frame->reference_line_info().front().path_decision();

  bool has_blocking_obstacle = false;
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (obstacle->IsVirtual() || !obstacle->IsStatic()) {
      continue;
    }
    CHECK(obstacle->IsStatic());
    if (obstacle->speed() >
        GetContext()->scenario_config_.block_obstacle_min_speed()) {
      continue;
    }
    if (obstacle->PerceptionSLBoundary().start_s() <=
        adc_sl_boundary.end_s()) {  // such vehicles are behind the ego car.
      continue;
    }
    if (obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() +
            GetContext()->scenario_config_.max_front_obstacle_distance()) {
      // vehicles are far away
      continue;
    }

    // check driving_width
    const auto& reference_line =
        frame->reference_line_info().front().reference_line();
    const double driving_width =
        reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());
    const double adc_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width();
    if (driving_width - adc_width - FLAGS_static_decision_nudge_l_buffer >
        GetContext()->scenario_config_.min_l_nudge_buffer()) {
      continue;
    }

    has_blocking_obstacle = true;
    break;
  }

  GetContext()->backup_stage_cycle_num_ += 1;
  if (!has_blocking_obstacle ||
      GetContext()->backup_stage_cycle_num_ >
          GetContext()->scenario_config_.max_backup_stage_cycle_num()) {
    GetContext()->backup_stage_cycle_num_ = 0;
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }

  // do path planning
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_start_point, frame);
  if (!plan_ok) {
    AERROR << "Stage " << Name() << " error: "
           << "planning on reference line failed.";
    return Stage::ERROR;
  }
  return Stage::RUNNING;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
