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
#include <vector>

#include "modules/planning/toolkits/deciders/destination.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneSegment;

Destination::Destination(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

Status Destination::ApplyRule(Frame* frame,
                              ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!frame->is_near_destination()) {
    return Status::OK();
  }

  const auto& routing =
      AdapterManager::GetRoutingResponse()->GetLatestObserved();

  common::SLPoint dest_sl;
  const auto& ref_line = reference_line_info->reference_line();
  const auto& routing_end = *routing.routing_request().waypoint().rbegin();
  ref_line.XYToSL({routing_end.pose().x(), routing_end.pose().y()}, &dest_sl);
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  const auto& dest = GetPlanningStatus()->destination();
  if (adc_sl.start_s() > dest_sl.s() && !dest.has_passed_destination()) {
    ADEBUG << "Destination at back, but we have not reached destination yet";
    return Status::OK();
  }

  BuildStopDecision(frame, reference_line_info);

  return Status::OK();
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
    AERROR << "routing_request has no end";
    return -1;
  }

  const auto* planning_status = GetPlanningStatus();
  const auto& routing_end = *routing.routing_request().waypoint().rbegin();
  double dest_lane_s =
      std::max(0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
                        config_.destination().stop_distance());

  if (planning_status->has_pull_over() &&
      planning_status->pull_over().has_status() &&
      planning_status->pull_over().status() == PullOverStatus::DISABLED) {
    Stop(frame, reference_line_info, routing_end.id(), dest_lane_s);
    ADEBUG << "destination: STOP at current lane. PULL-OVER disabled";
    return 0;
  }

  common::PointENU dest_point;
  if (CheckPullOver(reference_line_info, routing_end.id(), dest_lane_s,
                    &dest_point)) {
    if (planning_status->has_pull_over() &&
        planning_status->pull_over().in_pull_over()) {
      PullOver(nullptr);
      ADEBUG << "destination: continue PULL OVER";
    } else {
      PullOver(&dest_point);
      ADEBUG << "destination: PULL OVER";
    }
  } else {
    Stop(frame, reference_line_info, routing_end.id(), dest_lane_s);
    ADEBUG << "destination: STOP at current lane";
  }

  return 0;
}

/**
 * @brief: build on-lane stop decision upon arriving at destination
 */
int Destination::Stop(Frame* const frame,
                      ReferenceLineInfo* const reference_line_info,
                      const std::string lane_id, const double lane_s) {
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
  if (!reference_line.IsOnLane(stop_wall_box.center())) {
    ADEBUG << "destination point is not on lane";
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

/**
 * @brief: check if adc will pull-over upon arriving destination
 */
bool Destination::CheckPullOver(ReferenceLineInfo* const reference_line_info,
                                const std::string& dest_lane_id,
                                const double dest_lane_s,
                                common::PointENU* dest_point) {
  CHECK_NOTNULL(reference_line_info);

  if (!config_.destination().enable_pull_over()) {
    return false;
  }

  const auto hdmap_ptr = HDMapUtil::BaseMapPtr();
  if (!hdmap_ptr) {
    AERROR << "Invalid HD Map.";
    return false;
  }
  const auto dest_lane = hdmap_ptr->GetLaneById(hdmap::MakeMapId(dest_lane_id));
  if (!dest_lane) {
    AERROR << "Failed to find lane[" << dest_lane_id << "]";
    return false;
  }

  // check if ChangeLane
  bool change_lane = false;
  const auto& segments = reference_line_info->Lanes();
  if (segments.NextAction() != routing::FORWARD) {
    change_lane = true;
  }

  // check dest OnRoad
  const auto& reference_line = reference_line_info->reference_line();
  *dest_point = dest_lane->GetSmoothPoint(dest_lane_s);
  if (!change_lane && !reference_line.IsOnLane(*dest_point)) {
    return false;
  }

  // check dest within pull_over_plan_distance
  common::SLPoint dest_sl;
  if (!reference_line.XYToSL({dest_point->x(), dest_point->y()}, &dest_sl)) {
    AERROR << "failed to project the dest point to the other reference line";
    return false;
  }
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double distance_to_dest = dest_sl.s() - adc_front_edge_s;
  ADEBUG << "adc_front_edge_s[" << adc_front_edge_s << "] distance_to_dest["
         << distance_to_dest << "] dest_lane[" << dest_lane_id
         << "] dest_lane_s[" << dest_lane_s << "]";

  if (distance_to_dest > config_.destination().pull_over_plan_distance()) {
    // to far, not sending pull-over yet
    return false;
  }

  // Disable pull-over for the rest route if ChangeLane and clost to dest
  if (change_lane) {
    auto* planning_status = GetPlanningStatus();
    planning_status->clear_pull_over();
    auto* pull_over = planning_status->mutable_pull_over();
    pull_over->set_reason(PullOverStatus::DESTINATION);
    pull_over->set_status(PullOverStatus::DISABLED);
    pull_over->set_status_set_time(Clock::NowInSeconds());

    return false;
  }

  return true;
}

/**
 * @brief: build pull-over decision upon arriving at destination
 */
int Destination::PullOver(common::PointENU* const dest_point) {
  auto* planning_status = GetPlanningStatus();
  if (!planning_status->has_pull_over() ||
      !planning_status->pull_over().in_pull_over()) {
    planning_status->clear_pull_over();
    auto* pull_over = planning_status->mutable_pull_over();
    pull_over->set_in_pull_over(true);
    pull_over->set_reason(PullOverStatus::DESTINATION);
    pull_over->set_status_set_time(Clock::NowInSeconds());

    if (dest_point) {
      pull_over->mutable_inlane_dest_point()->set_x(dest_point->x());
      pull_over->mutable_inlane_dest_point()->set_y(dest_point->y());
    }
  }

  return 0;
}

}  // namespace planning
}  // namespace apollo
