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

#include "modules/planning/scenarios/side_pass/stage_generate_path.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;

/*
 * @brief:
 * STAGE: SidePassGeneratePath
 */
Stage::StageStatus StageGeneratePath::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  ADEBUG << "SIDEPASS: Generating path.";
  if (!ExecuteTaskOnReferenceLine(planning_start_point, frame)) {
    AERROR << "Fail to plan on reference_line.";
    GetContext()->backup_stage_cycle_num_ = 0;
    next_stage_ = ScenarioConfig::SIDE_PASS_BACKUP;
    return Stage::FINISHED;
  }
  GetContext()->path_data_ = frame->reference_line_info().front().path_data();
  if (frame->reference_line_info().front().trajectory().NumOfPoints() > 0) {
    next_stage_ = ScenarioConfig::SIDE_PASS_STOP_ON_WAITPOINT;
    return Stage::FINISHED;
  }
  return Stage::RUNNING;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
