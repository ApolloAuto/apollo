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

BacksideVehicle::BacksideVehicle(const RuleConfig& config)
    : TrafficRule(config) {}

void BacksideVehicle::MakeLaneKeepingObstacleDecision(
    const SLBoundary& adc_sl_boundary, PathDecision* path_decision) {
  ObjectDecisionType ignore;
  ignore.mutable_ignore();
  const std::string rule_id = RuleConfig::RuleId_Name(config_.rule_id());
  const double adc_length_s =
      adc_sl_boundary.end_s() - adc_sl_boundary.start_s();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    if (path_obstacle->perception_sl_boundary().end_s() >=
        adc_sl_boundary.end_s()) {  // don't ignore such vehicles.
      continue;
    }

    if (path_obstacle->st_boundary().IsEmpty()) {
      path_decision->AddLongitudinalDecision(rule_id, path_obstacle->Id(),
                                             ignore);
      path_decision->AddLateralDecision(rule_id, path_obstacle->Id(), ignore);
      continue;
    }
    // Ignore the car comes from back of ADC
    if (path_obstacle->st_boundary().min_s() < -adc_length_s) {
      path_decision->AddLongitudinalDecision(rule_id, path_obstacle->Id(),
                                             ignore);
      path_decision->AddLateralDecision(rule_id, path_obstacle->Id(), ignore);
      continue;
    }

    if (path_obstacle->perception_sl_boundary().start_s() <
        adc_sl_boundary.end_s()) {
      path_decision->AddLongitudinalDecision(rule_id, path_obstacle->Id(),
                                             ignore);
      path_decision->AddLateralDecision(rule_id, path_obstacle->Id(), ignore);
      continue;
    }
  }
}

void BacksideVehicle::MakeChangeLaneObstacleDecision(
    const SLBoundary& adc_sl_boundary, PathDecision* path_decision) {
  ObjectDecisionType overtake;
  const std::string rule_id = RuleConfig::RuleId_Name(config_.rule_id());
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const auto& sl_boundary = path_obstacle->perception_sl_boundary();
    if (sl_boundary.start_s() > adc_sl_boundary.end_s()) {
      // car is in front of us.
      continue;
    }
    double overtake_distance =
        std::max(FLAGS_overtake_min_distance,
                 path_obstacle->obstacle()->Speed() * FLAGS_overtake_min_time);
    if (sl_boundary.end_s() >= adc_sl_boundary.start_s() - overtake_distance) {
      // overlap with our car on in blind zone, cancel change lane.,
      reference_line_info_->SetDriable(false);
      ADEBUG << "Reference line " << reference_line_info_->Lanes().Id()
             << " is not drivable";
      return;
    }
  }
}

bool BacksideVehicle::ApplyRule(Frame*,
                                ReferenceLineInfo* const reference_line_info) {
  reference_line_info_ = reference_line_info;
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  if (reference_line_info->IsChangeLanePath()) {
    MakeChangeLaneObstacleDecision(adc_sl_boundary, path_decision);
  } else {
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision);
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
