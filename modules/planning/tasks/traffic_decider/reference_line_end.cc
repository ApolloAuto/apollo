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

ReferenceLineEnd::ReferenceLineEnd(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

bool ReferenceLineEnd::ApplyRule(Frame* frame,
                                 ReferenceLineInfo* const reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();
  // check
  double remain_s =
      reference_line.Length() - reference_line_info->AdcSlBoundary().end_s();
  const double& velocity = frame->vehicle_state().linear_velocity();
  const double stop_acc =
      std::fabs(common::VehicleConfigHelper::GetConfig()
                    .vehicle_param()
                    .max_deceleration()) *
      config_.reference_line_end().stop_acc_to_max_deceleration_ratio();
  const double stop_s = velocity * velocity / (2.0 * stop_acc) +
                        FLAGS_virtual_stop_wall_length +
                        config_.reference_line_end().stop_distance();
  if (stop_s < remain_s &&
      remain_s >
          config_.reference_line_end().min_reference_line_remain_length()) {
    ADEBUG << "have enough reference line to drive on";
    return true;
  }

  // create avirtual stop wall at the end of reference line to stop the adc
  std::string virtual_obstacle_id =
      REF_LINE_END_VO_ID_PREFIX + reference_line_info->Lanes().Id();
  double obstacle_start_s =
      reference_line.Length() - FLAGS_virtual_stop_wall_length;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, obstacle_start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
    return false;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
    return false;
  }

  // build stop decision
  auto stop_point = reference_line.GetReferencePoint(
      obstacle_start_s - config_.reference_line_end().stop_distance());
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);
  stop_decision->set_distance_s(-config_.reference_line_end().stop_distance());
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return true;
}

}  // namespace planning
}  // namespace apollo
