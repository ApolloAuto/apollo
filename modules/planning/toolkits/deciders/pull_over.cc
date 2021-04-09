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

#include "modules/planning/toolkits/deciders/pull_over.h"

#include <algorithm>
#include <iomanip>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/proto/map_lane.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/toolkits/deciders/util.h"

namespace apollo {
namespace planning {

using apollo::common::PointENU;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;
using apollo::perception::PerceptionObstacle;

uint32_t PullOver::failure_count_ = 0;
PointENU PullOver::stop_point_;
PointENU PullOver::inlane_stop_point_;

PullOver::PullOver(const TrafficRuleConfig& config) : TrafficRule(config) {}

Status PullOver::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;

  if (!IsPullOver()) {
    return Status::OK();
  }

  if (CheckPullOverComplete()) {
    return Status::OK();
  }

  common::PointENU stop_point;
  if (GetPullOverStopPoint(&stop_point) != 0) {
    BuildInLaneStop(stop_point);
    ADEBUG << "Could not find a safe pull over point. STOP in-lane";
  } else {
    BuildPullOverStop(stop_point);
  }

  return Status::OK();
}

/**
 * @brief: check if in pull_over state
 */
bool PullOver::IsPullOver() const {
  auto* planning_status = GetPlanningStatus();
  return (planning_status->has_pull_over() &&
          planning_status->pull_over().in_pull_over());
}

PullOver::ValidateStopPointCode PullOver::IsValidStop(
    const PointENU& stop_point) const {
  const auto& reference_line = reference_line_info_->reference_line();

  common::SLPoint stop_point_sl;
  reference_line.XYToSL(stop_point, &stop_point_sl);

  return IsValidStop(stop_point_sl);
}

PullOver::ValidateStopPointCode PullOver::IsValidStop(
    const common::SLPoint& stop_point_sl) const {
  const auto& reference_line = reference_line_info_->reference_line();
  if (stop_point_sl.s() < 0 || stop_point_sl.s() > reference_line.Length()) {
    return OUT_OF_REFERENCE_LINE;
  }

  // note: this check has to be done first
  const auto& pull_over_status = GetPlanningStatus()->pull_over();
  if (pull_over_status.has_inlane_dest_point()) {
    common::SLPoint dest_point_sl;
    reference_line.XYToSL({pull_over_status.inlane_dest_point().x(),
                           pull_over_status.inlane_dest_point().y()},
                          &dest_point_sl);
    if (stop_point_sl.s() - dest_point_sl.s() >
        config_.pull_over().max_check_distance()) {
      return PASS_DEST_POINT_TOO_FAR;
    }
  }

  const double adc_end_edge_s = reference_line_info_->AdcSlBoundary().start_s();
  if (stop_point_sl.s() <= adc_end_edge_s) {
    return BEHIND_ADC;
  }

  if (pull_over_status.status() != PullOverStatus::IN_OPERATION) {
    const double adc_front_edge_s =
        reference_line_info_->AdcSlBoundary().end_s();
    if (stop_point_sl.s() - adc_front_edge_s <
        config_.pull_over().operation_length()) {
      return OPERATION_LENGTH_NOT_ENOUGH;
    }
  }

  // parking spot boundary
  const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_width = vehicle_param.width();
  const double adc_length = vehicle_param.length();

  SLBoundary parking_spot_boundary;
  parking_spot_boundary.set_start_s(stop_point_sl.s() - adc_length -
                                    PARKING_SPOT_LONGITUDINAL_BUFFER);
  parking_spot_boundary.set_end_s(stop_point_sl.s() +
                                  PARKING_SPOT_LONGITUDINAL_BUFFER);
  parking_spot_boundary.set_start_l(stop_point_sl.l() - adc_width / 2 -
                                    config_.pull_over().buffer_to_boundary());
  parking_spot_boundary.set_end_l(stop_point_sl.l() + adc_width / 2);
  ADEBUG << "parking_spot_boundary: " << parking_spot_boundary.DebugString();

  // check obstacles
  auto* path_decision = reference_line_info_->path_decision();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const PerceptionObstacle& perception_obstacle =
        path_obstacle->obstacle()->Perception();
    const std::string& obstacle_id = std::to_string(perception_obstacle.id());
    PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
    std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle_type);

    if (path_obstacle->obstacle()->IsVirtual() ||
        !path_obstacle->obstacle()->IsStatic()) {
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] VIRTUAL or NOT STATIC. SKIP";
      continue;
    }

    const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
    if (!(parking_spot_boundary.start_s() > obstacle_sl.end_s() ||
          obstacle_sl.start_s() > parking_spot_boundary.end_s() ||
          parking_spot_boundary.start_l() > obstacle_sl.end_l() ||
          obstacle_sl.start_l() > parking_spot_boundary.end_l())) {
      // overlap
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] overlap with parking spot: " << obstacle_sl.DebugString();

      return PARKING_SPOT_NOT_AVAIL;
    }
  }

  return OK;
}

/**
 * @brief:get pull_over points(start & stop)
 */
int PullOver::GetPullOverStopPoint(PointENU* stop_point) {
  const auto& pull_over_status = GetPlanningStatus()->pull_over();

  if (inlane_stop_point_.has_x() && inlane_stop_point_.has_y()) {
    // if inlane_stop_point already set
    // use that instead of looking for valid pull over spot
    return -1;
  }

  bool found = false;
  bool retry = true;
  if (pull_over_status.has_start_point() && pull_over_status.has_stop_point()) {
    // reuse existing/previously-set stop point
    stop_point->set_x(pull_over_status.stop_point().x());
    stop_point->set_y(pull_over_status.stop_point().y());

    ValidateStopPointCode ret = IsValidStop(*stop_point);
    if (ret == OK) {
      found = true;
    } else {
      if (ret == PASS_DEST_POINT_TOO_FAR) {
        retry = false;
      } else if (failure_count_ < config_.pull_over().max_failure_count()) {
        retry = false;
      }
    }
  }

  if (!found && retry) {
    // finding pull_over_stop_point
    if (FindPullOverStop(stop_point) == 0) {
      found = true;
    }
  }

  // found valid pull_over_stop_point
  if (found) {
    failure_count_ = 0;
    stop_point_.set_x(stop_point->x());
    stop_point_.set_y(stop_point->y());
    return 0;
  }

  // when fail, use previous invalid stop_point for smoothness
  failure_count_++;
  if (stop_point_.has_x() && stop_point_.has_y() &&
      failure_count_ < config_.pull_over().max_failure_count()) {
    stop_point->set_x(stop_point_.x());
    stop_point->set_y(stop_point_.y());
    return 0;
  }

  // fail to find valid pull_over_stop_point
  stop_point_.Clear();
  return -1;
}

/**
 * @brief: check if s is on overlaps
 */
bool PullOver::OnOverlap(const double start_s, const double end_s) {
  const auto& reference_line = reference_line_info_->reference_line();

  // crosswalk
  const std::vector<PathOverlap>& crosswalk_overlaps =
      reference_line.map_path().crosswalk_overlaps();
  for (const auto& crosswalk_overlap : crosswalk_overlaps) {
    if (start_s <= crosswalk_overlap.end_s &&
        end_s >= crosswalk_overlap.start_s) {
      ADEBUG << "s[" << start_s << ", " << end_s << "] on crosswalk_overlap["
             << crosswalk_overlap.object_id << "] s["
             << crosswalk_overlap.start_s << ", " << crosswalk_overlap.end_s
             << "]";
      return true;
    }
  }

  // junction
  const std::vector<PathOverlap>& junction_overlaps =
      reference_line.map_path().junction_overlaps();
  for (const auto& junction_overlap : junction_overlaps) {
    if (start_s <= junction_overlap.end_s &&
        end_s >= junction_overlap.start_s) {
      ADEBUG << "s[" << start_s << ", " << end_s << "] on junction_overlap["
             << junction_overlap.object_id << "] s[" << junction_overlap.start_s
             << ", " << junction_overlap.end_s << "]";
      return true;
    }
  }

  // clear_area
  const std::vector<PathOverlap>& clear_area_overlaps =
      reference_line.map_path().clear_area_overlaps();
  for (const auto& clear_area_overlap : clear_area_overlaps) {
    if (start_s <= clear_area_overlap.end_s &&
        end_s >= clear_area_overlap.start_s) {
      ADEBUG << "s[" << start_s << ", " << end_s << "] on clear_area_overlap["
             << clear_area_overlap.object_id << "] s["
             << clear_area_overlap.start_s << ", " << clear_area_overlap.end_s
             << "]";
      return true;
    }
  }

  // speed_bump
  const std::vector<PathOverlap>& speed_bump_overlaps =
      reference_line.map_path().speed_bump_overlaps();
  for (const auto& speed_bump_overlap : speed_bump_overlaps) {
    if (start_s <= speed_bump_overlap.end_s &&
        end_s >= speed_bump_overlap.start_s) {
      ADEBUG << "s[" << start_s << ", " << end_s << "] on speed_bump_overlap["
             << speed_bump_overlap.object_id << "] s["
             << speed_bump_overlap.start_s << ", " << speed_bump_overlap.end_s
             << "]";
      return true;
    }
  }

  return false;
}

/**
 * @brief: find pull over location(start & stop
 */
int PullOver::FindPullOverStop(const double stop_point_s,
                               PointENU* stop_point) {
  const auto& reference_line = reference_line_info_->reference_line();
  if (stop_point_s < 0 || stop_point_s > reference_line.Length()) {
    return -1;
  }

  // find road_right_width
  const auto& vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_width = vehicle_param.width();
  const double adc_length = vehicle_param.length();

  double road_left_width = 0.0;
  double road_right_width = 0.0;

  const double parking_spot_end_s =
      stop_point_s + PARKING_SPOT_LONGITUDINAL_BUFFER;
  reference_line.GetRoadWidth(parking_spot_end_s, &road_left_width,
                              &road_right_width);
  const double parking_spot_end_s_road_right_width = road_right_width;

  const double adc_center_s = stop_point_s - adc_length / 2;
  reference_line.GetRoadWidth(adc_center_s, &road_left_width,
                              &road_right_width);
  const double adc_center_s_road_right_width = road_right_width;

  const double parking_spot_start_s =
      stop_point_s - adc_length - PARKING_SPOT_LONGITUDINAL_BUFFER;
  reference_line.GetRoadWidth(parking_spot_start_s, &road_left_width,
                              &road_right_width);
  const double parking_spot_start_s_road_right_width = road_right_width;

  road_right_width = std::min(std::min(parking_spot_end_s_road_right_width,
                                       adc_center_s_road_right_width),
                              parking_spot_start_s_road_right_width);

  common::SLPoint stop_point_sl;
  stop_point_sl.set_s(stop_point_s);
  stop_point_sl.set_l(-(road_right_width - adc_width / 2 -
                        config_.pull_over().buffer_to_boundary()));

  if (IsValidStop(stop_point_sl) == OK) {
    common::math::Vec2d point;
    reference_line.SLToXY(stop_point_sl, &point);
    stop_point->set_x(point.x());
    stop_point->set_y(point.y());
    ADEBUG << "stop_point: " << stop_point->DebugString();
    return 0;
  }

  return -1;
}

int PullOver::FindPullOverStop(PointENU* stop_point) {
  const auto& reference_line = reference_line_info_->reference_line();
  const double adc_front_edge_s = reference_line_info_->AdcSlBoundary().end_s();

  double check_length = 0.0;
  double total_check_length = 0.0;
  double check_s = adc_front_edge_s;

  constexpr double kDistanceUnit = 5.0;
  while (check_s < reference_line.Length() &&
         total_check_length < config_.pull_over().max_check_distance()) {
    check_s += kDistanceUnit;
    total_check_length += kDistanceUnit;

    // find next_lane to check
    std::string prev_lane_id;
    std::vector<hdmap::LaneInfoConstPtr> lanes;
    reference_line.GetLaneFromS(check_s, &lanes);
    hdmap::LaneInfoConstPtr lane;
    for (auto temp_lane : lanes) {
      if (temp_lane->lane().id().id() == prev_lane_id) {
        continue;
      }
      lane = temp_lane;
      prev_lane_id = temp_lane->lane().id().id();
      break;
    }

    std::string lane_id = lane->lane().id().id();
    ADEBUG << "check_s[" << check_s << "] lane[" << lane_id << "]";

    // check turn type: NO_TURN/LEFT_TURN/RIGHT_TURN/U_TURN
    const auto& turn = lane->lane().turn();
    if (turn != hdmap::Lane::NO_TURN) {
      ADEBUG << "path lane[" << lane_id << "] turn[" << Lane_LaneTurn_Name(turn)
             << "] can't pull over";
      check_length = 0.0;
      continue;
    }

    // check rightmost driving lane:
    //   NONE/CITY_DRIVING/BIKING/SIDEWALK/PARKING
    bool rightmost_driving_lane = true;
    for (const auto& neighbor_lane_id :
         lane->lane().right_neighbor_forward_lane_id()) {
      const auto hdmap_ptr = HDMapUtil::BaseMapPtr();
      CHECK_NOTNULL(hdmap_ptr);
      const auto& neighbor_lane = hdmap_ptr->GetLaneById(neighbor_lane_id);
      if (!neighbor_lane) {
        ADEBUG << "Failed to find lane[" << neighbor_lane_id.id() << "]";
        continue;
      }
      const auto& lane_type = neighbor_lane->lane().type();
      if (lane_type == hdmap::Lane::CITY_DRIVING) {
        ADEBUG << "lane[" << lane_id << "]'s right neighbor forward lane["
               << neighbor_lane_id.id() << "] type["
               << Lane_LaneType_Name(lane_type) << "] can't pull over";
        rightmost_driving_lane = false;
        break;
      }
    }
    if (!rightmost_driving_lane) {
      check_length = 0.0;
      continue;
    }

    // check if on overlaps
    const auto& vehicle_param =
        VehicleConfigHelper::GetConfig().vehicle_param();
    const double adc_length = vehicle_param.length();
    const double parking_spot_start_s =
        check_s - adc_length - PARKING_SPOT_LONGITUDINAL_BUFFER;
    const double parking_spot_end_s =
        check_s + PARKING_SPOT_LONGITUDINAL_BUFFER;
    if (OnOverlap(parking_spot_start_s, parking_spot_end_s)) {
      ADEBUG << "lane[" << lane_id << "] on overlap.  can't pull over";
      check_length = 0.0;
      continue;
    }

    // all the lane checks have passed
    check_length += kDistanceUnit;
    ADEBUG << "check_length: " << check_length
           << "; plan_distance:" << config_.pull_over().plan_distance();
    if (check_length >= config_.pull_over().plan_distance()) {
      PointENU point;
      // check corresponding parking_spot
      if (FindPullOverStop(check_s, &point) != 0) {
        // parking_spot not valid/available
        check_length = 0.0;
        continue;
      }

      stop_point->set_x(point.x());
      stop_point->set_y(point.y());
      ADEBUG << "stop point: lane[" << lane->id().id() << "] ("
             << stop_point->x() << ", " << stop_point->y() << ")";

      return 0;
    }
  }

  return -1;
}

bool PullOver::CheckPullOverComplete() {
  double adc_speed =
      common::VehicleStateProvider::instance()->linear_velocity();
  if (adc_speed > config_.pull_over().max_stop_speed()) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return false;
  }

  const auto& pull_over_status = GetPlanningStatus()->pull_over();
  if (!pull_over_status.has_stop_point()) {
    return false;
  }

  PointENU stop_point;
  stop_point.set_x(pull_over_status.stop_point().x());
  stop_point.set_y(pull_over_status.stop_point().y());

  common::SLPoint stop_point_sl;
  const auto& reference_line = reference_line_info_->reference_line();
  reference_line.XYToSL(stop_point, &stop_point_sl);

  // check stop close enough to stop line of the stop_sign
  double adc_front_edge_s = reference_line_info_->AdcSlBoundary().end_s();
  if (stop_point_sl.s() - adc_front_edge_s >
      config_.pull_over().max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  // no stop fence if ADC fully pass stop line
  double adc_end_edge_s = reference_line_info_->AdcSlBoundary().start_s();
  if (adc_end_edge_s > stop_point_sl.s()) {
    GetPlanningStatus()->mutable_pull_over()->set_status(PullOverStatus::DONE);

    return true;
  }

  // ADC keep stopping at stop fence
  BuildPullOverStop(stop_point);
  return true;
}

bool PullOver::CheckStopDeceleration(const double stop_line_s) const {
  double stop_deceleration =
      util::GetADCStopDeceleration(reference_line_info_, stop_line_s,
                                   config_.pull_over().min_pass_s_distance());
  return (stop_deceleration <= config_.pull_over().max_stop_deceleration());
}

int PullOver::BuildPullOverStop(const PointENU& stop_point) {
  const auto& reference_line = reference_line_info_->reference_line();
  common::SLPoint stop_point_sl;
  reference_line.XYToSL(stop_point, &stop_point_sl);

  double stop_line_s = stop_point_sl.s() + config_.pull_over().stop_distance();
  double stop_point_heading =
      reference_line.GetReferencePoint(stop_point_sl.s()).heading();

  BuildStopDecision("", stop_line_s, stop_point, stop_point_heading);

  // record in PlanningStatus
  auto* pull_over_status = GetPlanningStatus()->mutable_pull_over();

  common::SLPoint start_point_sl;
  start_point_sl.set_s(stop_point_sl.s() -
                       config_.pull_over().operation_length());
  start_point_sl.set_l(0.0);
  common::math::Vec2d start_point;
  reference_line.SLToXY(start_point_sl, &start_point);
  pull_over_status->mutable_start_point()->set_x(start_point.x());
  pull_over_status->mutable_start_point()->set_y(start_point.y());
  pull_over_status->mutable_stop_point()->set_x(stop_point.x());
  pull_over_status->mutable_stop_point()->set_y(stop_point.y());
  pull_over_status->set_stop_point_heading(stop_point_heading);

  ADEBUG << "pull_over_status: " << pull_over_status->DebugString();

  return 0;
}

int PullOver::BuildInLaneStop(const PointENU& pull_over_stop_point) {
  const auto& reference_line = reference_line_info_->reference_line();

  const double adc_front_edge_s = reference_line_info_->AdcSlBoundary().end_s();

  common::SLPoint stop_point_sl;
  bool inlane_stop_point_set = false;

  // use inlane_stop_point if it's previously set
  if (inlane_stop_point_.has_x() && inlane_stop_point_.has_y()) {
    reference_line.XYToSL(inlane_stop_point_, &stop_point_sl);
    if (CheckStopDeceleration(stop_point_sl.s())) {
      inlane_stop_point_set = true;
      ADEBUG << "BuildInLaneStop using previously set inlane_stop_point: s["
             << stop_point_sl.s() << "] dist["
             << stop_point_sl.s() - adc_front_edge_s << "] POINT:("
             << inlane_stop_point_.x() << ", " << inlane_stop_point_.y() << ")";
    }
  }

  // use inlane_dest_point if there's one
  const auto& pull_over_status = GetPlanningStatus()->pull_over();
  if (pull_over_status.has_inlane_dest_point()) {
    reference_line.XYToSL({pull_over_status.inlane_dest_point().x(),
                           pull_over_status.inlane_dest_point().y()},
                          &stop_point_sl);
    if (CheckStopDeceleration(stop_point_sl.s())) {
      inlane_stop_point_set = true;
      ADEBUG << "BuildInLaneStop using inlane_dest_point: s["
             << stop_point_sl.s() << "] dist["
             << stop_point_sl.s() - adc_front_edge_s << "] POINT:"
             << pull_over_status.inlane_dest_point().DebugString();
    }
  }

  // use a point corresponding to pull_over_stop_point
  if (!inlane_stop_point_set) {
    if (pull_over_stop_point.has_x() && pull_over_stop_point.has_y()) {
      reference_line.XYToSL(
          {pull_over_stop_point.x(), pull_over_stop_point.y()}, &stop_point_sl);
      if (CheckStopDeceleration(stop_point_sl.s())) {
        inlane_stop_point_set = true;
        ADEBUG << "BuildInLaneStop using pull_over_stop_point: s["
               << stop_point_sl.s() << "] dist["
               << stop_point_sl.s() - adc_front_edge_s
               << "] POINT:" << pull_over_stop_point.DebugString();
      }
    }
  }

  // use adc + stop_distance(based on adc_speed + deceleration)
  // for new inlane_stop_point
  if (!inlane_stop_point_set) {
    double adc_speed =
        common::VehicleStateProvider::instance()->linear_velocity();
    double stop_distance = (adc_speed * adc_speed) /
                           (2 * config_.pull_over().max_stop_deceleration());
    stop_point_sl.set_s(adc_front_edge_s + stop_distance);
    CheckStopDeceleration(stop_point_sl.s());
    ADEBUG << "BuildInLaneStop: adc: s[" << stop_point_sl.s()
           << "] l[0.0] adc_front_edge_s[" << adc_front_edge_s << "]";
  }

  const auto& inlane_point =
      reference_line.GetReferencePoint(stop_point_sl.s());

  inlane_stop_point_.set_x(inlane_point.x());
  inlane_stop_point_.set_y(inlane_point.y());

  PointENU stop_point;
  stop_point.set_x(inlane_point.x());
  stop_point.set_y(inlane_point.y());
  double stop_line_s = stop_point_sl.s() + config_.pull_over().stop_distance();
  double stop_point_heading =
      reference_line.GetReferencePoint(stop_point_sl.s()).heading();

  BuildStopDecision(INLANE_STOP_VO_ID_POSTFIX, stop_line_s, stop_point,
                    stop_point_heading);

  GetPlanningStatus()->clear_pull_over();

  return 0;
}

int PullOver::BuildStopDecision(const std::string& vistual_obstacle_id_postfix,
                                const double stop_line_s,
                                const PointENU& stop_point,
                                const double stop_point_heading) {
  const auto& reference_line = reference_line_info_->reference_line();
  if (stop_line_s < 0 || stop_line_s > reference_line.Length()) {
    return -1;
  }

  // create virtual stop wall
  const auto& pull_over_reason = GetPlanningStatus()->pull_over().reason();
  std::string virtual_obstacle_id =
      PULL_OVER_VO_ID_PREFIX + PullOverStatus_Reason_Name(pull_over_reason) +
      vistual_obstacle_id_postfix;

  auto* obstacle = frame_->CreateStopObstacle(reference_line_info_,
                                              virtual_obstacle_id, stop_line_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
    return -1;
  }
  PathObstacle* stop_wall = reference_line_info_->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for " << virtual_obstacle_id;
    return -1;
  }

  // build stop decision
  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_PULL_OVER);
  stop_decision->set_distance_s(-config_.pull_over().stop_distance());
  stop_decision->set_stop_heading(stop_point_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info_->path_decision();
  // if (!path_decision->MergeWithMainStop(
  //        stop.stop(), stop_wall->Id(), reference_line,
  //        reference_line_info_->AdcSlBoundary())) {
  //  ADEBUG << "signal " << virtual_obstacle_id << " is not the closest stop.";
  //  return -1;
  // }

  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return 0;
}

}  // namespace planning
}  // namespace apollo
