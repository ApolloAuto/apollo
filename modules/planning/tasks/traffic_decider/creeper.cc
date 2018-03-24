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

#include "modules/planning/tasks/traffic_decider/creeper.h"

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/tasks/traffic_decider/util.h"

namespace apollo {
namespace planning {

using apollo::common::util::WithinBound;
using apollo::hdmap::PathOverlap;

Creeper::Creeper() {}

bool Creeper::Run(Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!in_use_) {
    ADEBUG << "Creeper is not in use, skipped.";
    return true;
  }

  // find next overlap
  auto* next_overlap =
      reference_line_info->reference_line().map_path().NextLaneOverlap(
          reference_line_info->AdcSlBoundary().end_s());
  if (next_overlap == nullptr) {
    ADEBUG << "No next lane overlap.";
    return true;
  }

  constexpr double kMaxCreepTargetDistance = 5.0;
  constexpr double kCreepStopDistance = 0.5;  // TODO(all): move to config
  return BuildStopDecision(*const_cast<PathOverlap*>(next_overlap),
                           kCreepStopDistance, kMaxCreepTargetDistance, frame,
                           reference_line_info);
}

bool Creeper::BuildStopDecision(const PathOverlap& overlap,
                                const double stop_buffer,
                                const double max_creeping_target_distance,
                                Frame* frame,
                                ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), overlap.start_s)) {
    ADEBUG << "stop_sign " << overlap.object_id << " is not on reference line";
    return true;
  }

  // create virtual stop wall
  std::string virtual_obstacle_id = "CREEPER_" + overlap.object_id;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, overlap.start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return false;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
    return false;
  }

  // build stop decision
  const double stop_s = overlap.start_s - stop_buffer;
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_CREEPER);
  stop_decision->set_distance_s(-stop_buffer);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision("Creeper", stop_wall->Id(), stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
