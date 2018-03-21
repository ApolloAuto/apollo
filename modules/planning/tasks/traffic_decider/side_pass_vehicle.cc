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

#include "modules/planning/tasks/traffic_decider/side_pass_vehicle.h"

#include <vector>

#include "modules/planning/proto/planning_status.pb.h"

#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;
using apollo::planning::util::GetPlanningStatus;

SidePassVehicle::SidePassVehicle(const TrafficRuleConfig& config)
    : TrafficRule(config), hdmap_(apollo::hdmap::HDMapUtil::BaseMapPtr()) {}

bool SidePassVehicle::UpdateSidePassStatus(
    const SLBoundary& adc_sl_boundary,
    const common::TrajectoryPoint& adc_planning_point,
    PathDecision* path_decision) {
  CHECK_NOTNULL(path_decision);
  bool has_blocking_obstacle =
      HasBlockingObstacle(adc_sl_boundary, *path_decision);

  auto* sidepass_state = GetPlanningStatus()->mutable_side_pass();

  if (!sidepass_state->has_status()) {
    sidepass_state->set_status(SidePassStatus::UNKNOWN);
  }
  auto status = sidepass_state->status();
  switch (status) {
    case SidePassStatus::UNKNOWN: {
      sidepass_state->set_status(SidePassStatus::DRIVING);
      break;
    }
    case SidePassStatus::DRIVING: {
      constexpr double kAdcStopSpeedThreshold = 0.1;  // unit: m/s
      if (has_blocking_obstacle &&
          adc_planning_point.v() < kAdcStopSpeedThreshold) {
        sidepass_state->set_status(SidePassStatus::WAIT);
        sidepass_state->set_wait_start_time(Clock::NowInSeconds());
      }
      break;
    }
    case SidePassStatus::WAIT: {
      if (has_blocking_obstacle) {
        double wait_start_time = sidepass_state->wait_start_time();
        if (Clock::NowInSeconds() - wait_start_time >
            config_.side_pass_vehicle().wait_time()) {
          // calculate if the left/right lane exist
          std::vector<hdmap::LaneInfoConstPtr> lanes;
          reference_line_->GetLaneFromS(
              (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) / 2.0,
              &lanes);
          if (lanes.empty()) {
            AERROR << "No valid lane found at s = "
                   << (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) /
                          2.0;
            return false;
          }
          bool enter_sidepass_mode = false;
          ObjectSidePass::Type side = ObjectSidePass::LEFT;
          if (lanes.size() >= 2) {
            // currently do not sidepass when lanes > 2 (usually at junctions).
          } else {
            auto& lane = lanes.front()->lane();
            if (lane.left_neighbor_forward_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::LEFT;
            }
            if (!enter_sidepass_mode &&
                lane.right_neighbor_forward_lane_id_size() > 0) {
              bool has_city_driving = false;
              for (auto& id : lane.right_neighbor_forward_lane_id()) {
                if (hdmap_->GetLaneById(id)->lane().type() ==
                    hdmap::Lane::CITY_DRIVING) {
                  has_city_driving = true;
                  break;
                }
              }
              if (has_city_driving) {
                enter_sidepass_mode = true;
                side = ObjectSidePass::RIGHT;
              }
            }
            if (!enter_sidepass_mode &&
                lane.left_neighbor_reverse_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::LEFT;
            }
            if (!enter_sidepass_mode &&
                lane.right_neighbor_reverse_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::RIGHT;
            }
          }
          if (enter_sidepass_mode) {
            sidepass_state->set_status(SidePassStatus::SIDEPASS);
            sidepass_state->clear_wait_start_time();
            sidepass_state->set_pass_side(side);
          }
        }
      } else {
        sidepass_state->set_status(SidePassStatus::DRIVING);
        sidepass_state->clear_wait_start_time();
      }
      break;
    }
    case SidePassStatus::SIDEPASS: {
      if (!has_blocking_obstacle) {
        sidepass_state->set_status(SidePassStatus::DRIVING);
      }
      break;
    }
    default:
      break;
  }
  return true;
}

// a blocking obstacle is an obstacle blocks the road when it is not blocked (by
// other obstacles or traffic rules)
bool SidePassVehicle::HasBlockingObstacle(const SLBoundary& adc_sl_boundary,
                                          const PathDecision& path_decision) {
  auto* sidepass_status = GetPlanningStatus()->mutable_side_pass();
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
    if (path_obstacle->PerceptionSLBoundary().start_l() > 1.0 ||
        path_obstacle->PerceptionSLBoundary().end_l() < -1.0) {
      continue;
    }

    bool is_blocked_by_others = false;
    for (const auto* other_obstacle : path_decision.path_obstacles().Items()) {
      if (other_obstacle->Id() == path_obstacle->Id()) {
        continue;
      }
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              path_obstacle->PerceptionSLBoundary().end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              path_obstacle->PerceptionSLBoundary().start_l()) {
        // not blocking the backside vehicle
        continue;
      }

      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                       path_obstacle->PerceptionSLBoundary().end_s();
      if (delta_s < 0.0 || delta_s > kAdcDistanceThreshold) {
        continue;
      } else {
        // TODO(All): fixed the segmentation bug for large vehicles, otherwise
        // the follow line will be problematic.
        // is_blocked_by_others = true; break;
      }
    }
    if (!is_blocked_by_others) {
      sidepass_status->set_pass_obstacle_id(path_obstacle->Id());
      return true;
    }
  }
  return false;
}

bool SidePassVehicle::MakeSidePassObstacleDecision(
    const SLBoundary& adc_sl_boundary,
    const common::TrajectoryPoint& adc_planning_point,
    PathDecision* path_decision) {
  if (!UpdateSidePassStatus(adc_sl_boundary, adc_planning_point,
                            path_decision)) {
    return false;
  }

  auto* sidepass_status = GetPlanningStatus()->mutable_side_pass();
  ADEBUG << sidepass_status->DebugString();

  if (sidepass_status->status() == SidePassStatus::SIDEPASS) {
    ObjectDecisionType sidepass;
    sidepass.mutable_sidepass();
    sidepass.mutable_sidepass()->set_type(sidepass_status->pass_side());
    path_decision->AddLateralDecision(
        "sidepass_vehicle", sidepass_status->pass_obstacle_id(), sidepass);
  }
  return true;
}

bool SidePassVehicle::ApplyRule(Frame* const,
                                ReferenceLineInfo* const reference_line_info) {
  if (FLAGS_use_navigation_mode) {
    // do not sidepass on highway.
    return true;
  }

  reference_line_ = &(reference_line_info->reference_line());
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& adc_planning_point = reference_line_info->AdcPlanningPoint();
  if (reference_line_info->Lanes()
          .IsOnSegment()) {  // The lane keeping reference line.
    if (!MakeSidePassObstacleDecision(adc_sl_boundary, adc_planning_point,
                                      path_decision)) {
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
