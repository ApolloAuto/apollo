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

SidePassSafety::SidePassSafety(const TaskConfig &config) : Decider(config) {
  SetName("SidePassSafety");
}

Status SidePassSafety::Process(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
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

  auto *obstacle =
      frame->CreateStopObstacle(reference_line_info, virtual_obstacle_id,
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
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();

  const auto &path_obstacles =
      reference_line_info->path_decision()->path_obstacles().Items();

  for (const auto *path_obstacle : path_obstacles) {
    // ignore virtual and static obstacles
    if (path_obstacle->obstacle()->IsVirtual() ||
        path_obstacle->obstacle()->IsStatic()) {
      continue;
    }
    // ignore the obstacles behind adc and is able to catch adc after 5 secs.
    if (path_obstacle->PerceptionSLBoundary().end_s() < adc_front_edge_s &&
        path_obstacle->st_boundary().min_t() >  5) {
      continue;
    }
    // not overlapped obstacles
    if (path_obstacle->st_boundary().IsEmpty()) {
      continue;
    }
    // if near side obstacles exist, it is unsafe.
    if (std::abs(path_obstacle->PerceptionSLBoundary().start_l()) <=
        Config().side_pass_safety_config().min_obstacle_lateral_distance() &&
        std::abs(path_obstacle->PerceptionSLBoundary().end_l()) <=
            Config().side_pass_safety_config().min_obstacle_lateral_distance()
        ) {
      return false;
    }
    // overlap is more than 5 meters
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
