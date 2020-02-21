/******************************************************************************
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
#include <vector>

#include "modules/planning/traffic_rules/destination.h"

#include "modules/map/proto/map_lane.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

Destination::Destination(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

Status Destination::ApplyRule(Frame* frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  MakeDecisions(frame, reference_line_info);

  return Status::OK();
}

/**
 * @brief: build stop decision
 */
int Destination::MakeDecisions(Frame* frame,
                               ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!frame->is_near_destination()) {
    return 0;
  }

  const auto& routing = frame->local_view().routing;
  if (routing->routing_request().waypoint_size() < 2) {
    AERROR << "routing_request has no end";
    return -1;
  }

  common::SLPoint dest_sl;
  const auto& reference_line = reference_line_info->reference_line();
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  reference_line.XYToSL(routing_end.pose(), &dest_sl);
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& dest =
      PlanningContext::Instance()->mutable_planning_status()->destination();
  if (adc_sl.start_s() > dest_sl.s() && !dest.has_passed_destination()) {
    ADEBUG << "Destination at back, but we have not reached destination yet";
    return 0;
  }

  const std::string stop_wall_id = FLAGS_destination_obstacle_id;
  const std::vector<std::string> wait_for_obstacle_ids;

  if (FLAGS_enable_scenario_pull_over) {
    const auto& pull_over_status =
        PlanningContext::Instance()->planning_status().pull_over();
    if (pull_over_status.has_position() &&
        pull_over_status.position().has_x() &&
        pull_over_status.position().has_y()) {
      // build stop decision based on pull-over position
      ADEBUG << "BuildStopDecision: pull-over position";
      common::SLPoint pull_over_sl;
      reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

      const double stop_line_s = pull_over_sl.s() +
                                 VehicleConfigHelper::GetConfig()
                                     .vehicle_param()
                                     .front_edge_to_center() +
                                 config_.destination().stop_distance();
      util::BuildStopDecision(
          stop_wall_id, stop_line_s, config_.destination().stop_distance(),
          StopReasonCode::STOP_REASON_PULL_OVER, wait_for_obstacle_ids,
          TrafficRuleConfig::RuleId_Name(config_.rule_id()), frame,
          reference_line_info);
      return 0;
    }
  }

  // build stop decision
  ADEBUG << "BuildStopDecision: destination";
  const double dest_lane_s =
      std::fmax(0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
                         config_.destination().stop_distance());
  util::BuildStopDecision(stop_wall_id, routing_end.id(), dest_lane_s,
                          config_.destination().stop_distance(),
                          StopReasonCode::STOP_REASON_DESTINATION,
                          wait_for_obstacle_ids,
                          TrafficRuleConfig::RuleId_Name(config_.rule_id()),
                          frame, reference_line_info);

  return 0;
}

}  // namespace planning
}  // namespace apollo
