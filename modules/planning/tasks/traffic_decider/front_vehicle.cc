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
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/proto/planning_status.pb.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;
using apollo::perception::PerceptionObstacle;
using apollo::planning::util::GetPlanningStatus;

FrontVehicle::FrontVehicle(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

bool FrontVehicle::ApplyRule(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  MakeDecisions(frame, reference_line_info);

  return true;
}

/**
 * @brief: make decision
 */
void FrontVehicle::MakeDecisions(Frame* frame,
                             ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  MakeSidePassDecision(reference_line_info);

  MakeStopDecision(reference_line_info);
}

/**
 * @brief: make SIDEPASS decision
 */
bool FrontVehicle::MakeSidePassDecision(
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  if (FLAGS_use_navigation_mode) {
    // no SIDE_PASS in navigation mode
    return true;
  }

  if (!reference_line_info->Lanes().IsOnSegment()) {
    // The lane keeping reference line
    return true;
  }

  if (!ProcessSidePass(reference_line_info)) {
    return false;
  }

  auto* sidepass_status = GetPlanningStatus()->mutable_side_pass();
  if (sidepass_status->status() == SidePassStatus::SIDEPASS) {
    ADEBUG << "SIDEPASS: obstacle["
        << sidepass_status->pass_obstacle_id() << "]";
    ObjectDecisionType sidepass;
    auto sidepass_decision = sidepass.mutable_sidepass();
    sidepass_decision->set_type(sidepass_status->pass_side());

    auto* path_decision = reference_line_info->path_decision();
    path_decision->AddLateralDecision(
        "front_vehicle", sidepass_status->pass_obstacle_id(), sidepass);
  }

  return true;
}

bool FrontVehicle::ProcessSidePass(
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  // find obstacle being blocked, to process SIDEPASS
  std::string blocked_obstacle_id = FindBlockedObstacle(reference_line_info);

  auto* sidepass_status = GetPlanningStatus()->mutable_side_pass();
  if (!sidepass_status->has_status()) {
    sidepass_status->set_status(SidePassStatus::UNKNOWN);
  }
  auto status = sidepass_status->status();
  ADEBUG << "side_pass status: " << SidePassStatus_Status_Name(status);

  switch (status) {
    case SidePassStatus::UNKNOWN: {
      sidepass_status->set_status(SidePassStatus::DRIVING);
      break;
    }
    case SidePassStatus::DRIVING: {
      constexpr double kAdcStopSpeedThreshold = 0.1;  // unit: m/s
      const auto& adc_planning_point = reference_line_info->AdcPlanningPoint();
      if (!blocked_obstacle_id.empty() &&
          adc_planning_point.v() < kAdcStopSpeedThreshold) {
        sidepass_status->set_status(SidePassStatus::WAIT);
        sidepass_status->set_wait_start_time(Clock::NowInSeconds());
      }
      break;
    }
    case SidePassStatus::WAIT: {
      const auto& reference_line = reference_line_info->reference_line();
      const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();

      if (blocked_obstacle_id.empty()) {
        sidepass_status->set_status(SidePassStatus::DRIVING);
        sidepass_status->clear_wait_start_time();
      } else {
        double wait_start_time = sidepass_status->wait_start_time();
        double wait_time = Clock::NowInSeconds() - wait_start_time;
        ADEBUG << "wait_start_time: " << wait_start_time
               << "; wait_time: " << wait_time << "]";

        if (wait_time > config_.front_vehicle().side_pass_wait_time()) {
          // calculate if the left/right lane exist
          std::vector<hdmap::LaneInfoConstPtr> lanes;
          const double adc_s = (adc_sl_boundary.start_s() +
              adc_sl_boundary.end_s()) / 2.0;
          reference_line.GetLaneFromS(adc_s, &lanes);
          if (lanes.empty()) {
            AWARN << "No valid lane found at s[" << adc_s << "]";
            return false;
          }

          bool enter_sidepass_mode = false;
          ObjectSidePass::Type side = ObjectSidePass::LEFT;
          if (lanes.size() >= 2) {
            // currently do not sidepass when lanes > 2 (usually at junctions).
          } else {
            sidepass_status->set_status(SidePassStatus::DRIVING);
            sidepass_status->clear_wait_start_time();

            auto& lane = lanes.front()->lane();
            if (lane.left_neighbor_forward_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::LEFT;
            }
            if (!enter_sidepass_mode &&
                lane.right_neighbor_forward_lane_id_size() > 0) {
              bool has_city_driving = false;
              for (auto& id : lane.right_neighbor_forward_lane_id()) {
                if (HDMapUtil::BaseMap().GetLaneById(id)->lane().type() ==
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
            sidepass_status->set_status(SidePassStatus::SIDEPASS);
            sidepass_status->set_pass_obstacle_id(blocked_obstacle_id);
            sidepass_status->clear_wait_start_time();
            sidepass_status->set_pass_side(side);
          }
        }
      }
      break;
    }
    case SidePassStatus::SIDEPASS: {
      if (blocked_obstacle_id.empty()) {
        sidepass_status->set_status(SidePassStatus::DRIVING);
      }
      break;
    }
    default:
      break;
  }
  return true;
}

/**
 * @brief: a blocked obstacle is a static obstacle being blocked by
 *         other obstacles or traffic rules
 */
std::string& FrontVehicle::FindBlockedObstacle(
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  std::string blocked_obstacle_id;
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  auto* path_decision = reference_line_info->path_decision();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const PerceptionObstacle& perception_obstacle =
        path_obstacle->obstacle()->Perception();
    const std::string& obstacle_id = std::to_string(perception_obstacle.id());
    PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
    std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle_type);

    if (path_obstacle->obstacle()->IsVirtual() ||
        !path_obstacle->obstacle()->IsStatic()) {
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name
          << "] VIRTUAL or NOT STATIC. SKIP";
      continue;
    }

    if (path_obstacle->PerceptionSLBoundary().start_s() <=
        adc_sl_boundary.end_s()) {  // such vehicles are behind the adc.
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name << "] behind ADC. SKIP";
      continue;
    }

    constexpr double kAdcDistanceThreshold = 15.0;  // unit: m
    if (path_obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() +
            kAdcDistanceThreshold) {  // vehicles are far away
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name << "] too far. SKIP";
      continue;
    }

    if (path_obstacle->PerceptionSLBoundary().start_l() > 1.0 ||
        path_obstacle->PerceptionSLBoundary().end_l() < -1.0) {
      // TODO(all)
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name << "] nudgable. SKIP";
      continue;
    }

    bool is_blocked_by_others = false;
    for (const auto* other_obstacle : path_decision->path_obstacles().Items()) {
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
      blocked_obstacle_id = path_obstacle->Id();
    }
  }
  return blocked_obstacle_id;
}

void FrontVehicle::MakeStopDecision(
    ReferenceLineInfo* reference_line_info) {
  const auto& adc_sl = reference_line_info->AdcSlBoundary();
  auto* path_decision = reference_line_info->path_decision();
  const auto& reference_line = reference_line_info->reference_line();
  const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_width = vehicle_param.width();

  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const PerceptionObstacle& perception_obstacle =
        path_obstacle->obstacle()->Perception();
    const std::string& obstacle_id = std::to_string(perception_obstacle.id());
    PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
    std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle_type);

    if (path_obstacle->obstacle()->IsVirtual() ||
        !path_obstacle->obstacle()->IsStatic()) {
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name
          << "] VIRTUAL or NOT STATIC. SKIP";
      continue;
    }

    if (path_obstacle->PerceptionSLBoundary().end_s() <=
        adc_sl.start_s()) {  // skip backside vehicles
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name << "] behind ADC. SKIP";
      continue;
    }

    // check SIDE_PASS decision
    if (path_obstacle->LateralDecision().has_sidepass()) {
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name << "] SIDE_PASS. SKIP";
      continue;
    }

    const auto& sl = path_obstacle->PerceptionSLBoundary();
    double left_width = 0.0;
    double right_width = 0.0;
    reference_line.GetLaneWidth(sl.start_s(), &left_width, &right_width);

    double left_driving_width =
        left_width - sl.end_l() - FLAGS_static_decision_nudge_l_buffer;
    double right_driving_width =
        right_width + sl.start_l() - FLAGS_static_decision_nudge_l_buffer;

    ADEBUG << "obstacle_id[" << obstacle_id
        << "] type[" << obstacle_type_name
        << "] left_driving_width[" << left_driving_width
        << "] right_driving_width[" << right_driving_width
        << "] adc_width[" << adc_width
        << "] left[" << left_width << "] right_width[" << right_width << "]";

    // stop if not able to bypass or if obstacle crossed reference line
    if ((left_driving_width < adc_width && right_driving_width < adc_width) ||
        (sl.start_l() <= 0.0 && sl.end_l() >= 0.0)) {
      ADEBUG << "STOP: obstacle[" << obstacle_id << "]";
      // build stop decision
      const double stop_distance =
          path_obstacle->MinRadiusStopDistance(vehicle_param);
      const double stop_s = sl.start_s() - stop_distance;
      auto stop_point = reference_line.GetReferencePoint(stop_s);
      double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

      ObjectDecisionType stop;
      auto stop_decision = stop.mutable_stop();
      stop_decision->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
      stop_decision->set_distance_s(-stop_distance);
      stop_decision->set_stop_heading(stop_heading);
      stop_decision->mutable_stop_point()->set_x(stop_point.x());
      stop_decision->mutable_stop_point()->set_y(stop_point.y());
      stop_decision->mutable_stop_point()->set_z(0.0);

      path_decision->AddLongitudinalDecision(
          "front_vehicle", path_obstacle->Id(), stop);
    }
  }
}

}  // namespace planning
}  // namespace apollo
