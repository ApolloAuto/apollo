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
#include <algorithm>
#include <limits>

#include "modules/planning/tasks/traffic_decider/destination.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::hdmap::HDMapUtil;
using apollo::planning::util::GetPlanningStatus;

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
 * @brief: make decision
 */
void Destination::MakeDecisions(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (FLAGS_use_navigation_mode) {
    return;
  }

  if (!frame->is_near_destination()) {
    return;
  }

  BuildStopDecision(frame, reference_line_info);

  return;
}

/**
 * @brief: build stop decision
 */
int Destination::BuildStopDecision(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const auto& routing =
      AdapterManager::GetRoutingResponse()->GetLatestObserved();
  if (routing.routing_request().waypoint_size() < 2) {
    ADEBUG << "routing_request has no end";
    return -1;
  }

  const auto& routing_end = *routing.routing_request().waypoint().rbegin();
  double dest_lane_s = std::max(
      0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
      config_.destination().stop_distance());

  if (CheckPullOver(reference_line_info, routing_end.id(), dest_lane_s)) {
    PullOver();
  } else {
    Stop(frame, reference_line_info, routing_end.id(), dest_lane_s);
  }

  return 0;
}

int Destination::Stop(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info,
                      const std::string lane_id,
                      const double lane_s) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  const auto& reference_line = reference_line_info->reference_line();

  // create virtual stop wall
  std::string stop_wall_id = FLAGS_destination_obstacle_id;
  auto* obstacle = frame->CreateStopObstacle(stop_wall_id, lane_id, lane_s);
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
  const auto stop_wall_box = stop_wall->obstacle()->PerceptionBoundingBox();
  if (!reference_line.IsOnRoad(stop_wall_box.center())) {
    ADEBUG << "destination point is not on road";
    return 0;
  }
  auto stop_point = reference_line.GetReferencePoint(
      stop_wall->PerceptionSLBoundary().start_s() -
      config_.destination().stop_distance());

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);
  stop_decision->set_distance_s(-config_.destination().stop_distance());
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return 0;
}

bool Destination::CheckPullOver(
    ReferenceLineInfo* const reference_line_info,
    const std::string lane_id,
    const double lane_s) {
  CHECK_NOTNULL(reference_line_info);

  if (!config_.destination().enable_pull_over()) {
    return false;
  }

  const auto dest_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
      hdmap::MakeMapId(lane_id));
  if (!dest_lane) {
    AERROR << "Failed to find lane[" << lane_id << "]";
    return false;
  }
  double dest_lane_s = std::max(
      0.0, lane_s - FLAGS_virtual_stop_wall_length -
      config_.destination().stop_distance());
  auto dest_point = dest_lane->GetSmoothPoint(dest_lane_s);

  const auto& reference_line = reference_line_info->reference_line();

  double distance_to_dest = std::numeric_limits<double>::max();
  if (reference_line.IsOnRoad(dest_point)) {
    common::SLPoint dest_sl;
    if (!reference_line.XYToSL({dest_point.x(), dest_point.y()}, &dest_sl)) {
      AERROR << "failed to project the dest point to the other reference line";
      return false;
    }
    double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
    distance_to_dest = dest_sl.s() - adc_front_edge_s;
  }
  if (distance_to_dest <= config_.destination().star_distance_to_sp()) {
    return true;
  }

  return false;
}

int Destination::PullOver() {
  GetPlanningStatus()->mutable_pull_over()->set_in_pull_over(true);
  return 0;
}

}  // namespace planning
}  // namespace apollo
