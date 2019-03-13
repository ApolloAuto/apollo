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

/**
 * @file
 **/

#include "modules/planning/scenarios/side_pass/stage_side_pass.h"

#include <utility>

#include "modules/common/time/time.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using common::Status;
using common::TrajectoryPoint;
using common::time::Clock;

StageSidePass::StageSidePass(const ScenarioConfig::StageConfig& config) :
    Stage(config) {}

Stage::StageStatus StageSidePass::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  if (frame->mutable_reference_line_info()->empty()) {
    AERROR << "No reference line available for side pass stage";
    return StageStatus::ERROR;
  }
  auto& reference_line_info = frame->mutable_reference_line_info()->front();
  if (reference_line_info.IsChangeLanePath()) {
    AERROR << "Cannot perform side pass on a change-lane reference line!";
    return StageStatus::ERROR;
  }

  auto tasks_status = ExecuteTasks(planning_init_point, frame,
      &reference_line_info);

  if (tasks_status != Status::OK()) {
    auto fallback_status = PlanFallbackTrajectory(planning_init_point,
        frame, &reference_line_info);

    if (fallback_status != Status::OK()) {
      AERROR << "computing fallback trajectory failed";
      return Stage::StageStatus::ERROR;
    }
  }

  return Stage::StageStatus::RUNNING;
}

Status StageSidePass::ExecuteTasks(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {

  for (auto* ptr_task : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();
    auto task_status = ptr_task->Execute(frame, reference_line_info);
    if (!task_status.ok()) {
      AERROR << "Failed to run tasks[" << ptr_task->Name()
             << "], Error message: " << task_status.error_message();
      return Status(common::ErrorCode::PLANNING_ERROR);
    }
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000.0;

    ADEBUG << "after optimizer " << ptr_task->Name() << ":"
           << reference_line_info->PathSpeedDebugString() << std::endl;
    ADEBUG << ptr_task->Name() << " time spend: " << time_diff_ms << " ms.";
  }

  return Status::OK();
}

// TODO(all): merge the fallback strategy from all scenarios
Status StageSidePass::PlanFallbackTrajectory(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  // path and trajectory fall-back
  // TODO(all): finish later
  if (reference_line_info->path_data().Empty()) {
  } else if (reference_line_info->speed_data().empty()) {
  }
  return Status::OK();
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
