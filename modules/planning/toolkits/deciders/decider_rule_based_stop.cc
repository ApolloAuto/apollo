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

#include "modules/planning/toolkits/deciders/decider_rule_based_stop.h"

#include <string>

#include "modules/common/util/util.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::util::WithinBound;
using apollo::hdmap::PathOverlap;

DeciderRuleBasedStop::DeciderRuleBasedStop(
    const TaskConfig& config) : Decider(config) {
  CHECK(config.has_decider_rule_based_stop_config());
  SetName("DeciderRuleBasedStop");
}

Status DeciderRuleBasedStop::Process(Frame* frame,
                                ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  StopSign(frame, reference_line_info);

  TrafficLight(frame, reference_line_info);

  return Status::OK();
}

void DeciderRuleBasedStop::StopSign(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const std::string stop_sign_id =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.object_id;
  if (stop_sign_id.empty()) {
    return;
  }

  const std::string stop_wall_id = STOP_SIGN_VO_ID_PREFIX + stop_sign_id;
  const double stop_line_s =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.start_s;
  const double stop_distance =
      config_.decider_rule_based_stop_config().stop_distance();
  AERROR << "DeciderRuleBasedStop: stop_wall_id[" << stop_wall_id
      << "] stop_line_s[" << stop_line_s << "]";

  BuildStopDecision(frame, reference_line_info,
                    stop_wall_id,
                    stop_line_s,
                    stop_distance,
                    StopReasonCode::STOP_REASON_STOP_SIGN);
}

void DeciderRuleBasedStop::TrafficLight(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const std::string traffic_light_id =
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.object_id;
  if (traffic_light_id.empty()) {
    return;
  }

  // TODO(all): check traffic light

  const std::string stop_wall_id =
      TRAFFIC_LIGHT_VO_ID_PREFIX + traffic_light_id;
  const double stop_line_s =
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlap.start_s;
  const double stop_distance =
        config_.decider_rule_based_stop_config().stop_distance();

  AERROR << "DeciderRuleBasedStop: stop_wall_id[" << stop_wall_id
      << "] stop_line_s[" << stop_line_s << "]";
  BuildStopDecision(frame, reference_line_info,
                    stop_wall_id,
                    stop_line_s,
                    stop_distance,
                    StopReasonCode::STOP_REASON_SIGNAL);
}

bool DeciderRuleBasedStop::BuildStopDecision(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info,
    const std::string& stop_wall_id,
    const double stop_line_s,
    const double stop_distance,
    const StopReasonCode& stop_reason_code) {
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
  Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << stop_wall_id;
    return -1;
  }

  // build stop decision
  const double stop_s = stop_line_s - stop_distance;
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      "DeciderRuleBasedStop", stop_wall->Id(), stop);

  return 0;
}

}  // namespace planning
}  // namespace apollo
