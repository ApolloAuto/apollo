/*****************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/traffic_rules/stop_sign.h"

#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using common::Status;
using common::math::Vec2d;
using common::util::WithinBound;
using hdmap::PathOverlap;

StopSign::StopSign(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

Status StopSign::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void StopSign::MakeDecisions(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.stop_sign().enabled()) {
    return;
  }

  const double adc_back_edge_s =
      reference_line_info->AdcSlBoundary().start_s();

  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();
  for (const auto& stop_sign_overlap : stop_sign_overlaps) {
    if (stop_sign_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    const auto& stop_done_overlap_ids =
        PlanningContext::GetScenarioInfo()->stop_done_overlap_ids;
    if (stop_done_overlap_ids.end() !=
        std::find(stop_done_overlap_ids.begin(),
                  stop_done_overlap_ids.end(),
                  stop_sign_overlap.object_id)) {
      continue;
    }

    AERROR << "BuildStopDecision: stop_sign["
           << stop_sign_overlap.object_id
           << "] start_s[" << stop_sign_overlap.start_s << "]";
    BuildStopDecision(
        frame, reference_line_info, stop_sign_overlap,
        PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles);
  }
}

int StopSign::BuildStopDecision(
    Frame* const frame,
    ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap& stop_sign_overlap,
    const std::vector<std::string>& wait_for_obstacles) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(),
                   stop_sign_overlap.start_s)) {
    AERROR << "stop_line_s[" << stop_sign_overlap.start_s
           << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  std::string virtual_obstacle_id =
      STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, stop_sign_overlap.start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return -1;
  }
  Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create obstacle for: " << virtual_obstacle_id;
    return -1;
  }

  // build stop decision
  const double stop_s =
      stop_sign_overlap.start_s - config_.stop_sign().stop_distance();
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_STOP_SIGN);
  stop_decision->set_distance_s(-config_.stop_sign().stop_distance());
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return 0;
}

}  // namespace planning
}  // namespace apollo
