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

#include "modules/planning/toolkits/deciders/side_pass_safety.h"

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {

using ::apollo::common::ErrorCode;
using ::apollo::common::Status;
using ::apollo::hdmap::PathOverlap;

SidePassSafety::SidePassSafety() : Decider("SidePassSafety") {}

bool SidePassSafety::Init(const ScenarioConfig::ScenarioTaskConfig &config) {
  is_init_ = true;
  return true;
}

Status SidePassSafety::Process(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process().";
    return Status(ErrorCode::PLANNING_ERROR,
                  "SidePassSafety DeciderNot inited.");
  }

  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!IsSafeSidePass(frame, reference_line_info)) {
    BuildSidePathDecision(frame, reference_line_info);
  }

  return Status::OK();
}

apollo::common::Status SidePassSafety::BuildSidePathDecision(
    Frame *frame, ReferenceLineInfo *const reference_line_info) {
  std::string virtual_obstacle_id = SIDEPASS_VIRTUAL_OBSTACLE_ID;

  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id,
      reference_line_info->AdcSlBoundary().end_s());
  if (!obstacle) {
    AERROR << "Failed to create side pass safety obstacle["
           << virtual_obstacle_id << "]";
    auto status = Status(ErrorCode::PLANNING_ERROR,
               "Failed to create side pass safety obstacle");
    return status;
  }
  return Status().OK();
}

bool SidePassSafety::IsSafeSidePass(
    Frame *frame, ReferenceLineInfo *const reference_line_info) {
  const auto& path_obstacles =
      reference_line_info->path_decision()->path_obstacles().Items();

  for (const auto* path_obstacle : path_obstacles) {
    if (path_obstacle->obstacle()->IsVirtual()||
        !path_obstacle->obstacle()->IsStatic()) {
      continue;
    }
    if (path_obstacle->PerceptionSLBoundary().start_l() < 1.0 &&
        path_obstacle->PerceptionSLBoundary().end_l() > -1.0) {
      return false;
    }

    if (path_obstacle->st_boundary().IsEmpty()) {
      continue;
    }
    double s_range = path_obstacle->st_boundary().max_s() -
        path_obstacle->st_boundary().min_s();
    if (s_range > 5) {
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
