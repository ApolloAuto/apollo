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

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;

/*
 * @brief:
 * STAGE: StageBackup
 */
Stage::StageStatus StageBackup::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  // Check for front blocking obstacles.
  const auto& reference_line_info = frame->reference_line_info().front();
  const PathDecision& path_decision = reference_line_info.path_decision();
  bool exists_a_blocking_obstacle = false;
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (IsBlockingObstacleToSidePass(
            *frame, obstacle,
            GetContext()->scenario_config_.block_obstacle_min_speed(),
            GetContext()->scenario_config_.min_front_obstacle_distance(),
            GetContext()->scenario_config_.enable_obstacle_blocked_check())) {
      exists_a_blocking_obstacle = true;
      break;
    }
  }

  // If there is no more blocking obstacle or if we are in this stage
  // for too long, then exit stage.
  GetContext()->backup_stage_cycle_num_ += 1;
  if (!exists_a_blocking_obstacle ||
      GetContext()->backup_stage_cycle_num_ >
          GetContext()->scenario_config_.max_backup_stage_cycle_num()) {
    GetContext()->backup_stage_cycle_num_ = 0;
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }

  // Otherwise, do path planning.
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
