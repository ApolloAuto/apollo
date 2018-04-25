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

#include "modules/planning/tasks/traffic_decider/front_vehicle.h"

#include <string>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;

FrontVehicle::FrontVehicle(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

bool FrontVehicle::ApplyRule(Frame* frame,
                             ReferenceLineInfo* reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  StopForStaticCrossLaneObstacles(reference_line_info);
  return true;
}

void FrontVehicle::StopForStaticCrossLaneObstacles(
    ReferenceLineInfo* reference_line_info) {
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  auto* path_decision = reference_line_info->path_decision();
  const auto& ref_line = reference_line_info->reference_line();
  const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    if (path_obstacle->PerceptionSLBoundary().end_s() <=
        adc_sl.start_s()) {  // skip backside vehicles
      continue;
    }
    if (!path_obstacle->obstacle()->IsStatic()) {
      continue;
    }
    const auto& sl = path_obstacle->PerceptionSLBoundary();
    double left_width = 0.0;
    double right_width = 0.0;
    ref_line.GetLaneWidth(sl.start_s(), &left_width, &right_width);

    const double adc_width = vehicle_param.width();

    double left_driving_width =
        left_width - sl.end_l() - FLAGS_static_decision_nudge_l_buffer;
    double right_driving_width =
        right_width + sl.start_l() - FLAGS_static_decision_nudge_l_buffer;

    // stop if not able to bypass or if obstacle crossed reference line
    if ((left_driving_width < adc_width && right_driving_width < adc_width) ||
        (sl.start_l() <= 0.0 && sl.end_l() >= 0.0)) {
      ObjectDecisionType decision;
      auto* stop = decision.mutable_stop();
      stop->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
      stop->set_distance_s(
          -path_obstacle->MinRadiusStopDistance(vehicle_param));
      auto ref_point =
          ref_line.GetReferencePoint(sl.start_s() + stop->distance_s());
      stop->set_stop_heading(ref_point.heading());
      stop->mutable_stop_point()->set_x(ref_point.x());
      stop->mutable_stop_point()->set_y(ref_point.y());
      stop->mutable_stop_point()->set_z(0.0);
      path_decision->AddLongitudinalDecision("front_vehicle/crossed_ref_line",
                                             path_obstacle->Id(), decision);
    }
  }
}

}  // namespace planning
}  // namespace apollo
