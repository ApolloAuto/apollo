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

#include "modules/planning/tasks/traffic_decider/rerouting.h"

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;
using apollo::planning::util::GetPlanningStatus;

Rerouting::Rerouting(const TrafficRuleConfig& config) : TrafficRule(config) {}

bool Rerouting::ChangeLaneFailRerouting() {
  const auto& segments = reference_line_info_->Lanes();
  // 1. If current reference line is drive forward, no rerouting.
  if (segments.NextAction() == routing::FORWARD) {
    // if not current lane, do not check for rerouting
    return true;
  }

  // 2. If vehicle is not on current reference line yet, no rerouting
  if (!segments.IsOnSegment()) {
    return true;
  }

  // 3. If current reference line can connect to next passage, no rerouting
  if (segments.CanExit()) {
    return true;
  }

  // 4. If the end of current passage region not appeared, no rerouting
  const auto& route_end_waypoint = segments.RouteEndWaypoint();
  if (!route_end_waypoint.lane) {
    return true;
  }
  auto point = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
  const auto& reference_line = reference_line_info_->reference_line();
  common::SLPoint sl_point;
  if (!reference_line.XYToSL({point.x(), point.y()}, &sl_point)) {
    AERROR << "Failed to project point: " << point.ShortDebugString();
    return false;
  }
  if (!reference_line.IsOnRoad(sl_point)) {
    return true;
  }
  // 5. If the end of current passage region is further than kPrepareRoutingTime
  // * speed, no rerouting
  double adc_s = reference_line_info_->AdcSlBoundary().end_s();
  const auto* vehicle_state = common::VehicleStateProvider::instance();
  double speed = vehicle_state->linear_velocity();
  const double prepare_distance = speed * FLAGS_prepare_rerouting_time;
  if (sl_point.s() > adc_s + prepare_distance) {
    ADEBUG << "No need rerouting now because still can drive for time: "
           << FLAGS_prepare_rerouting_time << " seconds";
    return true;
  }
  // 6. Check if we have done rerouting before
  auto* rerouting = GetPlanningStatus()->mutable_rerouting();
  if (rerouting == nullptr) {
    AERROR << "rerouting is nullptr.";
    return false;
  }
  double current_time = Clock::NowInSeconds();
  if (rerouting->has_last_rerouting_time() &&
      (current_time - rerouting->last_rerouting_time() <
       config_.rerouting().cooldown_time())) {
    ADEBUG << "Skip rerouting and wait for previous rerouting result";
    return true;
  }
  if (!frame_->Rerouting()) {
    AERROR << "Failed to send rerouting request";
    return false;
  }

  // store last rerouting time.
  rerouting->set_last_rerouting_time(current_time);
  return true;
}

bool Rerouting::ApplyRule(Frame* const frame,
                          ReferenceLineInfo* const reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  if (!ChangeLaneFailRerouting()) {
    AERROR << "In un-successful lane change case, rerouting failed";
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
