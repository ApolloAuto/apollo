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

#include "modules/planning/tasks/traffic_decider/backside_vehicle.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

BacksideVehicle::BacksideVehicle(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

void BacksideVehicle::MakeLaneKeepingObstacleDecision(
    const SLBoundary& adc_sl_boundary, PathDecision* path_decision) {
  ObjectDecisionType ignore;
  ignore.mutable_ignore();
  const double adc_length_s =
      adc_sl_boundary.end_s() - adc_sl_boundary.start_s();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    if (path_obstacle->PerceptionSLBoundary().end_s() >=
        adc_sl_boundary.end_s()) {  // don't ignore such vehicles.
      continue;
    }

    if (path_obstacle->reference_line_st_boundary().IsEmpty()) {
      path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region",
                                             path_obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/no-st-region",
                                        path_obstacle->Id(), ignore);
      continue;
    }
    // Ignore the car comes from back of ADC
    if (path_obstacle->reference_line_st_boundary().min_s() < -adc_length_s) {
      path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc",
                                             path_obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc",
                                        path_obstacle->Id(), ignore);
      continue;
    }

    const double lane_boundary =
        config_.backside_vehicle().backside_lane_width();
    if (path_obstacle->PerceptionSLBoundary().start_s() <
        adc_sl_boundary.end_s()) {
      if (path_obstacle->PerceptionSLBoundary().start_l() > lane_boundary ||
          path_obstacle->PerceptionSLBoundary().end_l() < -lane_boundary) {
        continue;
      }
      path_decision->AddLongitudinalDecision("backside_vehicle/sl < adc.end_s",
                                             path_obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/sl < adc.end_s",
                                        path_obstacle->Id(), ignore);
      continue;
    }
  }
}

bool BacksideVehicle::ApplyRule(Frame* const,
                                ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  if (reference_line_info->Lanes()
          .IsOnSegment()) {  // The lane keeping reference line.
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision);
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
