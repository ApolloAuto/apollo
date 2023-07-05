/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/rule_based_stop_decider/rule_based_stop_decider.h"

#include <string>
#include <tuple>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/tasks/deciders/lane_change_decider/lane_change_decider.h"

namespace apollo {
namespace planning {

using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::math::Vec2d;

namespace {
// TODO(ALL): temporarily copy the value from lane_follow_stage.cc, will extract
// as a common value for planning later
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

RuleBasedStopDecider::RuleBasedStopDecider(
    const TaskConfig &config,
    const std::shared_ptr<DependencyInjector> &injector)
    : Decider(config, injector) {
  ACHECK(config.has_rule_based_stop_decider_config());
  rule_based_stop_decider_config_ = config.rule_based_stop_decider_config();
}

apollo::common::Status RuleBasedStopDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // 1. Rule_based stop for side pass onto reverse lane
  StopOnSidePass(frame, reference_line_info);

  // 2. Rule_based stop for urgent lane change
  if (FLAGS_enable_lane_change_urgency_checking) {
    CheckLaneChangeUrgency(frame);
  }

  // 3. Rule_based stop at path end position
  AddPathEndStop(frame, reference_line_info);

  return Status::OK();
}

void RuleBasedStopDecider::CheckLaneChangeUrgency(Frame *const frame) {
  for (auto &reference_line_info : *frame->mutable_reference_line_info()) {
    // Check if the target lane is blocked or not
    if (reference_line_info.IsChangeLanePath()) {
      is_clear_to_change_lane_ =
          LaneChangeDecider::IsClearToChangeLane(&reference_line_info);
      is_change_lane_planning_succeed_ =
          reference_line_info.Cost() < kStraightForwardLineCost;
      continue;
    }
    // If it's not in lane-change scenario || (target lane is not blocked &&
    // change lane planning succeed), skip
    if (frame->reference_line_info().size() <= 1 ||
        (is_clear_to_change_lane_ && is_change_lane_planning_succeed_)) {
      continue;
    }
    // When the target lane is blocked in change-lane case, check the urgency
    // Get the end point of current routing
    const auto &route_end_waypoint =
        reference_line_info.Lanes().RouteEndWaypoint();
    // If can't get lane from the route's end waypoint, then skip
    if (!route_end_waypoint.lane) {
      continue;
    }
    auto point = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
    auto *reference_line = reference_line_info.mutable_reference_line();
    common::SLPoint sl_point;
    // Project the end point to sl_point on current reference lane
    if (reference_line->XYToSL(point, &sl_point) &&
        reference_line->IsOnLane(sl_point)) {
      // Check the distance from ADC to the end point of current routing
      double distance_to_passage_end =
          sl_point.s() - reference_line_info.AdcSlBoundary().end_s();
      // If ADC is still far from the end of routing, no need to stop, skip
      if (distance_to_passage_end >
          rule_based_stop_decider_config_.approach_distance_for_lane_change()) {
        continue;
      }
      // In urgent case, set a temporary stop fence and wait to change lane
      // TODO(Jiaxuan Xu): replace the stop fence to more intelligent actions
      const std::string stop_wall_id = "lane_change_stop";
      std::vector<std::string> wait_for_obstacles;
      util::BuildStopDecision(
          stop_wall_id, sl_point.s(),
          rule_based_stop_decider_config_.urgent_distance_for_lane_change(),
          StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY, wait_for_obstacles,
          "RuleBasedStopDecider", frame, &reference_line_info);
    }
  }
}

void RuleBasedStopDecider::AddPathEndStop(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  if (!reference_line_info->path_data().path_label().empty() &&
      reference_line_info->path_data().frenet_frame_path().back().s() -
              reference_line_info->path_data().frenet_frame_path().front().s() <
          FLAGS_short_path_length_threshold) {
    const std::string stop_wall_id =
        PATH_END_VO_ID_PREFIX + reference_line_info->path_data().path_label();
    std::vector<std::string> wait_for_obstacles;
    util::BuildStopDecision(
        stop_wall_id,
        reference_line_info->path_data().frenet_frame_path().back().s() - 5.0,
        0.0, StopReasonCode::STOP_REASON_REFERENCE_END, wait_for_obstacles,
        "RuleBasedStopDecider", frame, reference_line_info);
  }
}

void RuleBasedStopDecider::StopOnSidePass(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  static bool check_clear;
  static common::PathPoint change_lane_stop_path_point;

  const PathData &path_data = reference_line_info->path_data();
  double stop_s_on_pathdata = 0.0;

  if (path_data.path_label().find("self") != std::string::npos) {
    check_clear = false;
    change_lane_stop_path_point.Clear();
    return;
  }

  if (check_clear &&
      CheckClearDone(*reference_line_info, change_lane_stop_path_point)) {
    check_clear = false;
  }

  if (!check_clear &&
      CheckSidePassStop(path_data, *reference_line_info, &stop_s_on_pathdata)) {
    if (!LaneChangeDecider::IsPerceptionBlocked(
            *reference_line_info,
            rule_based_stop_decider_config_.search_beam_length(),
            rule_based_stop_decider_config_.search_beam_radius_intensity(),
            rule_based_stop_decider_config_.search_range(),
            rule_based_stop_decider_config_.is_block_angle_threshold()) &&
        LaneChangeDecider::IsClearToChangeLane(reference_line_info)) {
      return;
    }
    if (!CheckADCStop(path_data, *reference_line_info, stop_s_on_pathdata)) {
      if (!BuildSidePassStopFence(path_data, stop_s_on_pathdata,
                                  &change_lane_stop_path_point, frame,
                                  reference_line_info)) {
        AERROR << "Set side pass stop fail";
      }
    } else {
      if (LaneChangeDecider::IsClearToChangeLane(reference_line_info)) {
        check_clear = true;
      }
    }
  }
}

// @brief Check if necessary to set stop fence used for nonscenario side pass
bool RuleBasedStopDecider::CheckSidePassStop(
    const PathData &path_data, const ReferenceLineInfo &reference_line_info,
    double *stop_s_on_pathdata) {
  const std::vector<std::tuple<double, PathData::PathPointType, double>>
      &path_point_decision_guide = path_data.path_point_decision_guide();
  PathData::PathPointType last_path_point_type =
      PathData::PathPointType::UNKNOWN;
  for (const auto &point_guide : path_point_decision_guide) {
    if (last_path_point_type == PathData::PathPointType::IN_LANE &&
        std::get<1>(point_guide) ==
            PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      *stop_s_on_pathdata = std::get<0>(point_guide);
      // Approximate the stop fence s based on the vehicle position
      const auto &vehicle_config =
          common::VehicleConfigHelper::Instance()->GetConfig();
      const double ego_front_to_center =
          vehicle_config.vehicle_param().front_edge_to_center();
      common::PathPoint stop_pathpoint;
      if (!path_data.GetPathPointWithRefS(*stop_s_on_pathdata,
                                          &stop_pathpoint)) {
        AERROR << "Can't get stop point on path data";
        return false;
      }
      const double ego_theta = stop_pathpoint.theta();
      Vec2d shift_vec{ego_front_to_center * std::cos(ego_theta),
                      ego_front_to_center * std::sin(ego_theta)};
      const Vec2d stop_fence_pose =
          shift_vec + Vec2d(stop_pathpoint.x(), stop_pathpoint.y());
      double stop_l_on_pathdata = 0.0;
      const auto &nearby_path = reference_line_info.reference_line().map_path();
      nearby_path.GetNearestPoint(stop_fence_pose, stop_s_on_pathdata,
                                  &stop_l_on_pathdata);
      return true;
    }
    last_path_point_type = std::get<1>(point_guide);
  }
  return false;
}

// @brief Set stop fence for side pass
bool RuleBasedStopDecider::BuildSidePassStopFence(
    const PathData &path_data, const double stop_s_on_pathdata,
    common::PathPoint *stop_point, Frame *const frame,
    ReferenceLineInfo *const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, stop_point)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  const std::string stop_wall_id = "Side_Pass_Stop";
  std::vector<std::string> wait_for_obstacles;

  const auto &nearby_path = reference_line_info->reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  nearby_path.GetNearestPoint({stop_point->x(), stop_point->y()}, &stop_point_s,
                              &stop_point_l);

  util::BuildStopDecision(stop_wall_id, stop_point_s, 0.0,
                          StopReasonCode::STOP_REASON_SIDEPASS_SAFETY,
                          wait_for_obstacles, "RuleBasedStopDecider", frame,
                          reference_line_info);
  return true;
}

// @brief Check if ADV stop at a stop fence
bool RuleBasedStopDecider::CheckADCStop(
    const PathData &path_data, const ReferenceLineInfo &reference_line_info,
    const double stop_s_on_pathdata) {
  common::PathPoint stop_point;
  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, &stop_point)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  const double adc_speed = injector_->vehicle_state()->linear_velocity();
  if (adc_speed > rule_based_stop_decider_config_.max_adc_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  // check stop close enough to stop line of the stop_sign
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const auto &nearby_path = reference_line_info.reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  nearby_path.GetNearestPoint({stop_point.x(), stop_point.y()}, &stop_point_s,
                              &stop_point_l);

  const double distance_stop_line_to_adc_front_edge =
      stop_point_s - adc_front_edge_s;

  if (distance_stop_line_to_adc_front_edge >
      rule_based_stop_decider_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  return true;
}

bool RuleBasedStopDecider::CheckClearDone(
    const ReferenceLineInfo &reference_line_info,
    const common::PathPoint &stop_point) {
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_start_l = reference_line_info.AdcSlBoundary().start_l();
  const double adc_end_l = reference_line_info.AdcSlBoundary().end_l();
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_info.reference_line().GetLaneWidth(
      (adc_front_edge_s + adc_back_edge_s) / 2.0, &lane_left_width,
      &lane_right_width);
  SLPoint stop_sl_point;
  reference_line_info.reference_line().XYToSL(stop_point, &stop_sl_point);
  // use distance to last stop point to determine if needed to check clear
  // again
  if (adc_back_edge_s > stop_sl_point.s()) {
    if (adc_start_l > -lane_right_width || adc_end_l < lane_left_width) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
