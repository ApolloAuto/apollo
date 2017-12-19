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

#include "modules/planning/tasks/traffic_decider/destination.h"

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

Destination::Destination(const RuleConfig& config) : TrafficRule(config) {}

bool Destination::ApplyRule(Frame*,
                            ReferenceLineInfo* const reference_line_info) {
  const auto& path_decision = reference_line_info->path_decision();
  auto* destination = path_decision->Find(FLAGS_destination_obstacle_id);
  if (!destination) {  // no destination in current reference line.
    return true;
  }

  const auto& box = destination->obstacle()->PerceptionBoundingBox();
  if (!reference_line_info->reference_line().IsOnRoad(box.center())) {
    return true;
  }

  auto stop_point = reference_line_info->reference_line().GetReferencePoint(
      destination->perception_sl_boundary().start_s() -
      FLAGS_stop_distance_destination);
  ObjectDecisionType stop;
  stop.mutable_stop();
  stop.mutable_stop()->set_distance_s(-FLAGS_stop_distance_destination);
  stop.mutable_stop()->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);
  stop.mutable_stop()->set_stop_heading(stop_point.heading());
  stop.mutable_stop()->mutable_stop_point()->set_x(stop_point.x());
  stop.mutable_stop()->mutable_stop_point()->set_y(stop_point.y());
  stop.mutable_stop()->mutable_stop_point()->set_z(0.0);
  path_decision->AddLongitudinalDecision(
      RuleConfig::RuleId_Name(config_.rule_id()), destination->Id(), stop);
  return true;
}

}  // namespace planning
}  // namespace apollo
