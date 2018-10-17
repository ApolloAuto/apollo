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

#include "modules/planning/toolkits/deciders/decider_stop_sign.h"

#include <string>

#include "modules/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::util::WithinBound;
using apollo::hdmap::PathOverlap;

DeciderStopSign::DeciderStopSign(const TaskConfig& config) : Decider(config) {
  CHECK(config.has_decider_stop_sign_config());
  SetName("DeciderCreep");
}

Status DeciderStopSign::Process(Frame* frame,
                                ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // TODO(all): to read from scenario context
  const std::string stop_sign_id = "TEMP";
  const double stop_line_s = 100;

  const double stop_distance =
      config_.decider_stop_sign_config().stop_distance();
  const std::string stop_wall_id = STOP_SIGN_VO_ID_PREFIX + stop_sign_id;
  BuildStopDecision(frame, reference_line_info, stop_wall_id, stop_line_s,
                    stop_distance);

  return Status::OK();
}

bool DeciderStopSign::BuildStopDecision(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const std::string& stop_wall_id, const double stop_line_s,
    const double stop_distance) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), stop_line_s)) {
    AERROR << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  auto* obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id, stop_line_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for: " << stop_wall_id;
    return -1;
  }

  // build stop decision
  const double stop_s = stop_line_s - stop_distance;
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_STOP_SIGN);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision("DeciderStopSign", stop_wall->Id(),
                                         stop);

  return 0;
}

}  // namespace planning
}  // namespace apollo
