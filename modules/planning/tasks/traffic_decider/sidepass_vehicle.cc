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

#include "modules/planning/tasks/traffic_decider/sidepass_vehicle.h"

#include "modules/common/time/time.h"
#include "modules/common/util/dropbox.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;
using apollo::common::util::Dropbox;

SidepassVehicle::SidepassVehicle(const RuleConfig& config)
    : TrafficRule(config) {}

void SidepassVehicle::MakeSidepassObstacleDecision(
    const SLBoundary& adc_sl_boundary,
    const common::TrajectoryPoint& adc_planning_point,
    PathDecision* path_decision) {
  constexpr double kAdcSpeedThreshold = 11.2;  // unit: m/s (25mph)
  if (adc_planning_point.v() > kAdcSpeedThreshold) {
    return;
  }

  bool need_sidepass = false;
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    if (path_obstacle->obstacle()->IsVirtual() ||
        !path_obstacle->obstacle()->IsStatic()) {
      continue;
    }
    CHECK(path_obstacle->obstacle()->IsStatic());

    if (path_obstacle->PerceptionSLBoundary().start_s() <=
        adc_sl_boundary.end_s()) {  // such vehicles are behind the adc.
      continue;
    }
    constexpr double kAdcDistanceThreshold = 15.0;  // unit: m
    if (path_obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() +
            kAdcDistanceThreshold) {  // vehicles are far away
      continue;
    }

    if (path_obstacle->PerceptionSLBoundary().start_l() < 0 &&
        path_obstacle->PerceptionSLBoundary().end_l() > 0) {
      // need sidepass, check wait_time
      bool is_waited_long_enough = false;

      double duration = 0.0;
      auto* wait_start_time =
          Dropbox<double>::Open()->Get(db_key_sidepass_adc_wait_start_time);
      if (wait_start_time != nullptr) {
        duration =
            Clock::NowInSeconds() - *(Dropbox<double>::Open()->Get(
                                        db_key_sidepass_adc_wait_start_time));
      }
      constexpr double kWaitDuration = 5.0;
      if (duration > kWaitDuration) {
        is_waited_long_enough = true;
      }

      constexpr double kAdcStopSpeedThreshold = 0.1;  // unit: m/s (25mph)
      if (adc_planning_point.v() < kAdcStopSpeedThreshold) {
        if (duration < kWaitDuration) {
          // only set new start time when vehicle is stopped.
          Dropbox<double>::Open()->Set(db_key_sidepass_adc_wait_start_time,
                                       Clock::NowInSeconds());
        }
      }

      if (is_waited_long_enough) {
        ObjectDecisionType sidepass;
        sidepass.mutable_sidepass();
        path_decision->AddLateralDecision("sidepass_vehicle",
                                          path_obstacle->Id(), sidepass);
        need_sidepass = true;
      }
    }
  }
  if (!need_sidepass) {
    // update start time
    Dropbox<double>::Open()->Remove(db_key_sidepass_adc_wait_start_time);
  }
}

bool SidepassVehicle::ApplyRule(Frame*,
                                ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& adc_planning_point = reference_line_info->AdcPlanningPoint();
  if (reference_line_info->Lanes()
          .IsOnSegment()) {  // The lane keeping reference line.
    MakeSidepassObstacleDecision(adc_sl_boundary, adc_planning_point,
                                 path_decision);
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
