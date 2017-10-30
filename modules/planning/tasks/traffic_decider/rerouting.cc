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
#include "modules/common/vehicle_state/vehicle_state.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

Rerouting::Rerouting(const RuleConfig& config) : TrafficRule(config) {}

bool Rerouting::ChangeLaneFailRerouting(
    const ReferenceLineInfo* reference_line_info) {
  const double kPrepareRoutingTime = 3.0;  // seconds
  const auto& segments = reference_line_info->Lanes();
  // 1. If current reference line is drive forward, no rerouting.
  if (segments.NextAction() == routing::FORWARD) {
    // if not current lane, do not check for rerouting
    ADEBUG << "This is a change lane target reference line, no need to check "
              "rerouting here.";
    return true;
  }
  // 2. If vehicle is not on current reference line yet, no rerouting
  if (!segments.IsOnSegment()) {
    ADEBUG << "This is a change lane target reference line, no need to check "
              "rerouting here.";
    return true;
  }

  // 3. If current reference line can connect to next passage, no rerouting
  if (segments.CanExit()) {
    ADEBUG << "This reference line can lead to another passage in routig";
    return true;
  }

  // 4. If the end of current passage region not appeared, no rerouting
  const auto& route_end_waypoint =
      reference_line_info->Lanes().RouteEndWaypoint();
  if (!route_end_waypoint.lane) {
    ADEBUG << "the end of passage is not visible yet, no need rerouting";
    return true;
  }
  // 5. If the end of current passage region not on reference line, no rerouting
  auto point = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
  const auto& reference_line = reference_line_info->reference_line();
  common::SLPoint sl_point;
  if (!reference_line.XYToSL({point.x(), point.y()}, &sl_point)) {
    AERROR << "Failed to project point: " << point.ShortDebugString();
    return false;
  }
  if (!reference_line.IsOnRoad(sl_point)) {
    ADEBUG << "end of passage not on road yet, no need rerouting";
    return true;
  }
  // 6. If the end of current passage region is further than kPrepareRoutingTime
  // * speed, no rerouting
  double adc_s = reference_line_info->AdcSlBoundary().end_s();
  const auto* vehicle_state = common::VehicleState::instance();
  double speed = vehicle_state->linear_velocity();
  const double prepare_distance = speed * kPrepareRoutingTime;
  if (sl_point.s() > adc_s + prepare_distance) {
    ADEBUG << "No need rerouting now because still can drive for time: "
           << kPrepareRoutingTime << " seconds";
    return true;
  }
  // Do rerouting.
  if (!Frame::Rerouting()) {
    AERROR << "Failed to send rerouting request";
    return false;
  }
  return true;
}

bool Rerouting::ApplyRule(Frame*,
                          ReferenceLineInfo* const reference_line_info) {
  if (!ChangeLaneFailRerouting(reference_line_info)) {
    AERROR << "In un-successful lane change case, rerouting failed";
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
