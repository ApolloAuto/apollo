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

#include "modules/planning/tasks/traffic_decider/traffic_decider.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/traffic_decider/back_side_vehicles.h"

namespace apollo {
namespace planning {
using ::apollo::common::Status;
using ::apollo::common::VehicleConfigHelper;

TrafficDecider::TrafficDecider() : Task("TrafficDecider") {}

void TrafficDecider::RegisterRules() {
  rule_factory_.Register("BackSideVehicles", []() -> TrafficRule * {
    return new BackSideVehicles();
  });
}

bool TrafficDecider::Init(const PlanningConfig &config) {
  RegisterRules();
  rule_configs_ = config.rule_config();
  is_init_ = true;
  return true;
}

const PathObstacle *TrafficDecider::CreateDestinationPathObstacle() {
  // set destination point

  const auto *destination_obstacle =
      frame_->FindObstacle(FLAGS_destination_obstacle_id);
  if (!destination_obstacle) {
    destination_obstacle = CreateDestinationObstacle();
    ADEBUG << "Created destination obstacle";
  }
  if (!destination_obstacle) {
    return nullptr;
  } else {
    const auto *ptr = reference_line_info_->AddObstacle(destination_obstacle);
    if (!ptr) {
      AERROR << "Failed to add destination obstacle's projection";
      return nullptr;
    }
    return ptr;
  }
}

const Obstacle *TrafficDecider::CreateDestinationObstacle() {
  const auto &routing_response = frame_->routing_response();
  if (!routing_response.routing_request().has_end()) {
    ADEBUG << "routing_request has no end";
    return nullptr;
  }
  common::math::Vec2d destination;
  destination.set_x(routing_response.routing_request().end().pose().x());
  destination.set_y(routing_response.routing_request().end().pose().y());

  // check if destination point is in planning range
  common::SLPoint destination_sl;
  const auto &reference_line = reference_line_info_->reference_line();
  reference_line.XYToSL(destination, &destination_sl);
  double destination_s = destination_sl.s();
  double destination_l = destination_sl.l();
  double left_bound;
  double right_bound;
  if (!reference_line.GetLaneWidth(destination_s, &left_bound, &right_bound)) {
    left_bound = right_bound = FLAGS_default_reference_line_width / 2;
  }
  if (destination_s < 0 || destination_s > reference_line.Length() ||
      destination_l > left_bound || destination_l < -right_bound) {
    AINFO << "destination[s=" << destination_s << "; l=" << destination_l
          << "] out of planning range. Skip";
    return nullptr;
  }

  // adjust destination based on adc front s
  if (destination_sl.s() <= reference_line_info_->AdcSlBoundary().end_s()) {
    destination_s = reference_line_info_->AdcSlBoundary().end_s() +
                    FLAGS_destination_adjust_distance_buffer;
  }

  std::unique_ptr<Obstacle> obstacle_ptr =
      reference_line_info_->CreateVirtualObstacle(
          FLAGS_destination_obstacle_id, {destination.x(), destination.y()},
          FLAGS_virtual_stop_wall_length, FLAGS_virtual_stop_wall_width,
          FLAGS_virtual_stop_wall_height);
  const auto *obstacle = obstacle_ptr.get();
  if (!frame_->AddObstacle(std::move(obstacle_ptr))) {
    AERROR << "Failed to add destination obstacle";
    return nullptr;
  }
  return obstacle;
}

Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);

  // 1. add destination stop
  if (!MakeDestinationStopDecision()) {
    AINFO << "There is no destination stop";
  } else {
    AINFO << "destination is created";
  }
  for (const auto rule_config : rule_configs_) {
    auto rule = rule_factory_.CreateObject(rule_config.name());
    if (!rule) {
      AERROR << "Could not find rule " << rule_config.DebugString();
      continue;
    }
    rule->ApplyRule(reference_line_info);
    AINFO << "Applied rule " << rule_config.name();
  }
  return Status::OK();
}

bool TrafficDecider::MakeDestinationStopDecision() {
  const auto *path_obstacle = CreateDestinationPathObstacle();
  if (!path_obstacle) {
    AINFO << "The path obstacle is not found";
    return false;
  }
  const auto *obstacle = path_obstacle->obstacle();
  const auto &reference_line = reference_line_info_->reference_line();

  // check stop_posision on reference line
  auto stop_position = obstacle->Perception().position();
  common::SLPoint stop_line_sl;
  reference_line.XYToSL({stop_position.x(), stop_position.y()}, &stop_line_sl);
  if (!reference_line.IsOnRoad(stop_line_sl)) {
    return false;
  }

  // check stop_line_s vs adc_s. stop_line_s must be ahead of adc_front_s
  if (stop_line_sl.s() <= reference_line_info_->AdcSlBoundary().end_s()) {
    ADEBUG << "skip: object:" << obstacle->Id() << " fence route_s["
           << stop_line_sl.s() << "] behind adc_front_s["
           << reference_line_info_->AdcSlBoundary().end_s() << "]";
    return false;
  }

  ObjectDecisionType object_stop;
  ObjectStop *object_stop_ptr = object_stop.mutable_stop();
  object_stop_ptr->set_distance_s(FLAGS_stop_line_min_distance);
  object_stop_ptr->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);

  auto stop_ref_point =
      reference_line.GetReferencePoint(stop_position.x(), stop_position.y());
  object_stop_ptr->mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop_ptr->mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop_ptr->set_stop_heading(stop_ref_point.heading());

  reference_line_info_->path_decision()->AddLongitudinalDecision(
      "Destination", obstacle->Id(), object_stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
