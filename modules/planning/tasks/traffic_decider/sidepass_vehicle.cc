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
    : TrafficRule(config) {
  Dropbox<SidepassStatus>::Open()->Set(db_key_sidepass_status,
                                       SidepassStatus::DRIVING);
}

void SidepassVehicle::UpdateSidepassStatus(
    const SLBoundary& adc_sl_boundary,
    const common::TrajectoryPoint& adc_planning_point,
    PathDecision* path_decision) {
  CHECK_NOTNULL(path_decision);
  bool has_blocking_obstacle =
      HasBlockingObstacle(adc_sl_boundary, *path_decision);

  auto* status = Dropbox<SidepassStatus>::Open()->Get(db_key_sidepass_status);
  CHECK_NOTNULL(status);

  switch (*status) {
    case SidepassStatus::DRIVING:
      if (has_blocking_obstacle) {
        constexpr double kAdcStopSpeedThreshold = 0.1;  // unit: m/s
        if (adc_planning_point.v() < kAdcStopSpeedThreshold) {
          Dropbox<SidepassStatus>::Open()->Set(db_key_sidepass_status,
                                               SidepassStatus::WAIT);
          Dropbox<double>::Open()->Set(db_key_sidepass_adc_wait_start_time,
                                       Clock::NowInSeconds());
        }
      }
      break;
    case SidepassStatus::WAIT:
      if (has_blocking_obstacle) {
        double* wait_start_time =
            Dropbox<double>::Open()->Get(db_key_sidepass_adc_wait_start_time);
        DCHECK_NOTNULL(wait_start_time);
        constexpr double kWaitDuration = 5.0;
        if (Clock::NowInSeconds() - *wait_start_time > kWaitDuration) {
          Dropbox<SidepassStatus>::Open()->Set(db_key_sidepass_status,
                                               SidepassStatus::SIDEPASS);
          Dropbox<double>::Open()->Remove(db_key_sidepass_adc_wait_start_time);
        }
      } else {
        Dropbox<SidepassStatus>::Open()->Set(db_key_sidepass_status,
                                             SidepassStatus::DRIVING);
        Dropbox<double>::Open()->Remove(db_key_sidepass_adc_wait_start_time);
      }
      break;
    case SidepassStatus::SIDEPASS:
      if (!has_blocking_obstacle) {
        Dropbox<SidepassStatus>::Open()->Set(db_key_sidepass_status,
                                             SidepassStatus::DRIVING);
      }
      break;
    default:
      break;
  }
}

bool SidepassVehicle::HasBlockingObstacle(const SLBoundary& adc_sl_boundary,
                                          const PathDecision& path_decision) {
  for (const auto* path_obstacle : path_decision.path_obstacles().Items()) {
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
      blocking_obstacle_id_ = path_obstacle->Id();
      return true;
    }
  }
  return false;
}

void SidepassVehicle::MakeSidepassObstacleDecision(
    const SLBoundary& adc_sl_boundary,
    const common::TrajectoryPoint& adc_planning_point,
    PathDecision* path_decision) {
  UpdateSidepassStatus(adc_sl_boundary, adc_planning_point, path_decision);

  auto* status = Dropbox<SidepassStatus>::Open()->Get(db_key_sidepass_status);
  DCHECK_NOTNULL(status);
  if (*status == SidepassStatus::SIDEPASS) {
    ObjectDecisionType sidepass;
    sidepass.mutable_sidepass();
    path_decision->AddLateralDecision("sidepass_vehicle", blocking_obstacle_id_,
                                      sidepass);
    blocking_obstacle_id_.clear();
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
