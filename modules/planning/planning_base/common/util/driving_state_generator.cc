/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/planning/planning_base/common/util/driving_state_generator.h"

#include "modules/planning/planning_base/common/path/path_data.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

void DrivingStateGenerator::generate(
    const ReferenceLineInfo* reference_line_info, const EgoInfo* ego_info,
    planning_internal::ADCDrivingState* const state) {
  if (nullptr != ego_info->adc_waypoint().lane) {
    state->set_lane_id(ego_info->adc_waypoint().lane->id().id());
    state->set_s_in_lane(ego_info->adc_waypoint().s);
  }

  state->set_distance_to_destination(ego_info->distance_to_destination());
  if (nullptr == reference_line_info) {
    AERROR << "DriveReferenceLineInfo is nullptr";
    return;
  }

  state->set_is_in_junction(
      planning::util::CheckInsideJunction(*reference_line_info));

  add_path_info(reference_line_info, state);
  add_lane_info(reference_line_info, state);
  add_nudge_info(reference_line_info, state);
}

void DrivingStateGenerator::add_path_info(
    const ReferenceLineInfo* reference_line_info,
    planning_internal::ADCDrivingState* const state) {
  state->set_lane_change_type(planning_internal::NO_CHANGE);
  state->set_lane_borrow_type(planning_internal::NO_BORROW);
  const PathData current_path_data = reference_line_info->path_data();
  if (reference_line_info->IsChangeLanePath() ||
      current_path_data.path_label().find("lane_change") != std::string::npos) {
    if (reference_line_info->Lanes().PreviousAction() ==
        routing::ChangeLaneType::LEFT) {
      state->set_lane_change_type(planning_internal::LEFT_CHANGE);
    } else if (reference_line_info->Lanes().PreviousAction() ==
               routing::ChangeLaneType::RIGHT) {
      state->set_lane_change_type(planning_internal::RIGHT_CHANGE);
    }
  } else {
    if (current_path_data.path_label().find("left") != std::string::npos) {
      state->set_lane_borrow_type(planning_internal::LEFT_BORROW);
    } else if (current_path_data.path_label().find("right") !=
               std::string::npos) {
      state->set_lane_borrow_type(planning_internal::RIGHT_BORROW);
    }
  }
  if (current_path_data.path_label().find("fallback") != std::string::npos) {
    state->set_is_fallback(true);
  }
}

void DrivingStateGenerator::add_lane_info(
    const ReferenceLineInfo* reference_line_info,
    planning_internal::ADCDrivingState* const state) {
  double adc_s = (reference_line_info->AdcSlBoundary().start_s() +
                  reference_line_info->AdcSlBoundary().end_s()) *
                 0.5;
  double route_lane_start_s = 0.0;
  double route_lane_end_s = 0.0;
  for (const auto& seg : reference_line_info->Lanes()) {
    const auto& turn_type = seg.lane->lane().turn();

    route_lane_start_s = route_lane_end_s;
    route_lane_end_s += seg.end_s - seg.start_s;

    AINFO << "s: " << route_lane_start_s << ", " << route_lane_end_s
          << ", ego s: " << adc_s;
    AINFO << "turn_type: " << turn_type;

    if (route_lane_end_s < adc_s) {
      continue;
    }
    if (route_lane_start_s < adc_s + 0.01) {
      state->set_cur_turn_info(seg.lane->lane().turn());
      continue;
    }
    if (turn_type == hdmap::Lane::LEFT_TURN ||
        turn_type == hdmap::Lane::RIGHT_TURN ||
        turn_type == hdmap::Lane::U_TURN) {
      state->mutable_next_turn_info()->set_turn_type(turn_type);
      state->mutable_next_turn_info()->set_dis_to_turn(route_lane_start_s -
                                                       adc_s);
      break;
    }
  }
}

void DrivingStateGenerator::add_nudge_info(
    const ReferenceLineInfo* reference_line_info,
    planning_internal::ADCDrivingState* const state) {
  SLBoundary adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto path_decision = reference_line_info->path_decision();
  double nearest_nudge_obs_dis = 1000.0;
  for (auto sl_polygon : reference_line_info->obs_sl_polygons()) {
    if (sl_polygon.NudgeInfo() == SLPolygon::IGNORE ||
        sl_polygon.NudgeInfo() == SLPolygon::UNDEFINED ||
        !sl_polygon.OverlapeWithReferCenter()) {
      continue;
    }

    const Obstacle* perception_obs = path_decision.Find(sl_polygon.id());
    if (!perception_obs) {
      AERROR << "Failed to find obstacle : " << sl_polygon.id();
      continue;
    }

    auto nudge_obstacle_debug = state->add_nudge_obstacle();
    nudge_obstacle_debug->set_is_blocked(false);
    if (sl_polygon.NudgeInfo() == SLPolygon::LEFT_NUDGE) {
      nudge_obstacle_debug->set_type(
          planning_internal::NudgeObstacleDebug::LEFT_NUDGE);
    } else if (sl_polygon.NudgeInfo() == SLPolygon::RIGHT_NUDGE) {
      nudge_obstacle_debug->set_type(
          planning_internal::NudgeObstacleDebug::RIGHT_NUDGE);
    } else if (sl_polygon.NudgeInfo() == SLPolygon::BLOCKED) {
      nudge_obstacle_debug->set_type(
          planning_internal::NudgeObstacleDebug::NO_NUDGE);
      nudge_obstacle_debug->set_is_blocked(true);
    }

    if (sl_polygon.MinS() - adc_sl_boundary.end_s() < nearest_nudge_obs_dis) {
      nearest_nudge_obs_dis = sl_polygon.MinS() - adc_sl_boundary.end_s();
    }

    nudge_obstacle_debug->mutable_obstacle()->set_id(perception_obs->Id());
    nudge_obstacle_debug->mutable_obstacle()->mutable_sl_boundary()->CopyFrom(
        perception_obs->PerceptionSLBoundary());
  }
  const auto& reference_line_towing_l =
      reference_line_info->reference_line_towing_l();
  double towing_l = 0.0;
  if (reference_line_towing_l.size() > 0) {
    towing_l = reference_line_towing_l.at(0);
  }
  if (nearest_nudge_obs_dis < 6.0 &&
      std::abs((adc_sl_boundary.start_l() + adc_sl_boundary.end_l()) * 0.5 -
               towing_l) > FLAGS_driving_state_nudge_check_l) {
    state->set_is_nudging(true);
  } else {
    state->set_is_nudging(false);
  }
}

}  // namespace planning
}  // namespace apollo
