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

#include "modules/planning/tasks/traffic_decider/reference_line_end.h"

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

ReferenceLineEnd::ReferenceLineEnd(const RuleConfig& config)
    : TrafficRule(config) {}

bool ReferenceLineEnd::ApplyRule(Frame* frame,
                                 ReferenceLineInfo* const reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();
  double remain_s =
      reference_line.Length() - reference_line_info->AdcSlBoundary().end_s();
  const double& velocity = frame->vehicle_state().linear_velocity();
  const double stop_acc = std::fabs(common::VehicleConfigHelper::GetConfig()
                                        .vehicle_param()
                                        .max_deceleration() /
                                    3.0);
  const double stop_s = velocity * velocity / (2.0 * stop_acc) +
                        FLAGS_virtual_stop_wall_length +
                        FLAGS_stop_distance_destination;
  if (stop_s < remain_s) {
    ADEBUG << "have enough reference line to drive on";
    return true;
  }
  // Need to create an obstacle at the end of reference line to stop the
  // vehicle
  double left_width = 0.0;
  double right_width = 0.0;
  double obstacle_start_s =
      reference_line.Length() - FLAGS_virtual_stop_wall_length;
  reference_line.GetLaneWidth(obstacle_start_s, &left_width, &right_width);

  auto center_point = reference_line.GetReferencePoint(
      obstacle_start_s + FLAGS_virtual_stop_wall_length / 2.0);

  common::math::Box2d stop_box{center_point, center_point.heading(),
                               FLAGS_virtual_stop_wall_length,
                               left_width + right_width};

  auto* obstacle = frame->AddStaticVirtualObstacle(
      FLAGS_reference_line_end_obstacle_id + "_" +
          reference_line_info->Lanes().Id(),
      stop_box);
  auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
  auto* path_decision = reference_line_info->path_decision();
  auto stop_point = reference_line.GetReferencePoint(
      obstacle_start_s - FLAGS_stop_distance_destination);
  ObjectDecisionType stop;
  stop.mutable_stop();
  stop.mutable_stop()->set_distance_s(-FLAGS_stop_distance_destination);
  stop.mutable_stop()->set_stop_heading(stop_point.heading());
  stop.mutable_stop()->mutable_stop_point()->set_x(stop_point.x());
  stop.mutable_stop()->mutable_stop_point()->set_y(stop_point.y());
  stop.mutable_stop()->mutable_stop_point()->set_z(0.0);
  path_decision->AddLongitudinalDecision(
      RuleConfig::RuleId_Name(config_.rule_id()), path_obstacle->Id(), stop);
  return true;
}

}  // namespace planning
}  // namespace apollo
