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

#include "modules/planning/tasks/deciders/decider_rule_based_stop.h"

#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::time::Clock;
using apollo::common::util::WithinBound;
using apollo::perception::TrafficLight;

DeciderRuleBasedStop::DeciderRuleBasedStop(const TaskConfig& config)
    : Decider(config) {
  CHECK(config.has_decider_rule_based_stop_config());
  SetName("DeciderRuleBasedStop");
}

Status DeciderRuleBasedStop::Process(Frame* frame,
                                     ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  CheckStopSign(frame, reference_line_info);

  CheckTrafficLight(frame, reference_line_info);

  return Status::OK();
}

void DeciderRuleBasedStop::CheckStopSign(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.decider_rule_based_stop_config().stop_sign().enabled()) {
    return;
  }

  const std::string stop_sign_id =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.object_id;
  if (stop_sign_id.empty()) {
    return;
  }

  if (PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.end() !=
      std::find(
          PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.begin(),
          PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.end(),
          stop_sign_id)) {
    return;
  }

  const std::string stop_wall_id = STOP_SIGN_VO_ID_PREFIX + stop_sign_id;
  const double stop_line_s =
      PlanningContext::GetScenarioInfo()->next_stop_sign_overlap.start_s;
  const double stop_distance =
      config_.decider_rule_based_stop_config().stop_sign().stop_distance();
  ADEBUG << "DeciderRuleBasedStop: stop_wall_id[" << stop_wall_id
         << "] stop_line_s[" << stop_line_s << "]";

  BuildStopDecision(
      frame, reference_line_info, stop_wall_id, stop_line_s, stop_distance,
      StopReasonCode::STOP_REASON_STOP_SIGN,
      PlanningContext::GetScenarioInfo()->stop_sign_wait_for_obstacles);
}

void DeciderRuleBasedStop::CheckTrafficLight(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.decider_rule_based_stop_config().traffic_light().enabled()) {
    return;
  }

  for (auto traffic_light_overlap :
      PlanningContext::GetScenarioInfo()->next_traffic_light_overlaps) {
    const std::string traffic_light_id = traffic_light_overlap.object_id;

    if (PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.end() !=
        std::find(
            PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.begin(),
            PlanningContext::GetScenarioInfo()->stop_done_overlap_ids.end(),
            traffic_light_id)) {
      continue;
    }


    auto signal_color = scenario::GetSignal(traffic_light_id).color();
    ADEBUG << "traffic_light_id[" << traffic_light_id
        << "] color[" << signal_color << "]";
    if (signal_color == TrafficLight::GREEN) {
      continue;
    }

    // TODO(all): add stop_deceleration check based on signal colors

    const std::string stop_wall_id =
        TRAFFIC_LIGHT_VO_ID_PREFIX + traffic_light_id;
    const double stop_line_s = traffic_light_overlap.start_s;
    const double stop_distance =
        config_.decider_rule_based_stop_config().traffic_light()
            .stop_distance();

    ADEBUG << "DeciderRuleBasedStop: stop_wall_id[" << stop_wall_id
        << "] stop_line_s[" << stop_line_s << "]";
    std::vector<std::string> wait_for_obstacles;
    BuildStopDecision(frame, reference_line_info, stop_wall_id, stop_line_s,
                      stop_distance, StopReasonCode::STOP_REASON_SIGNAL,
                      wait_for_obstacles);
  }
}

bool DeciderRuleBasedStop::BuildStopDecision(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const std::string& stop_wall_id, const double stop_line_s,
    const double stop_distance, const StopReasonCode& stop_reason_code,
    const std::vector<std::string>& wait_for_obstacles) {
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

  for (size_t i = 0; i < wait_for_obstacles.size(); ++i) {
    stop_decision->add_wait_for_obstacle(wait_for_obstacles[i]);
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision("DeciderRuleBasedStop",
                                         stop_wall->Id(), stop);

  return 0;
}

}  // namespace planning
}  // namespace apollo
