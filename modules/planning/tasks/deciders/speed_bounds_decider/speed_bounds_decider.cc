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

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.h"

#include <algorithm>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/change_lane_decider.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Vec2d;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

SpeedBoundsDecider::SpeedBoundsDecider(const TaskConfig &config)
    : Decider(config) {
  CHECK(config.has_speed_bounds_decider_config());
  speed_bounds_config_ = config.speed_bounds_decider_config();
  SetName("SpeedBoundsDecider");
}

Status SpeedBoundsDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // retrieve data from frame and reference_line_info
  const SLBoundary &adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const PathData &path_data = reference_line_info->path_data();
  const TrajectoryPoint &init_point = frame->PlanningStartPoint();
  const ReferenceLine &reference_line = reference_line_info->reference_line();
  PathDecision *const path_decision = reference_line_info->path_decision();

  AddPathEndStop(frame, reference_line_info);

  // 1. Rule_based speed planning configurations for different traffic scenarios
  if (FLAGS_enable_nonscenario_side_pass) {
    StopOnSidePass(frame, reference_line_info);
  }

  // Check the lane change urgency at current frame
  if (FLAGS_enable_lane_change_urgency_checking) {
    CheckLaneChangeUrgency(frame);
  }

  // 2. Map obstacles into st graph
  StBoundaryMapper boundary_mapper(adc_sl_boundary, speed_bounds_config_,
                                   reference_line, path_data,
                                   speed_bounds_config_.total_path_length(),
                                   speed_bounds_config_.total_time(),
                                   reference_line_info_->IsChangeLanePath());

  path_decision->EraseStBoundaries();
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==
      ErrorCode::PLANNING_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<const STBoundary *> boundaries;
  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &id = obstacle->Id();
    const auto &st_boundary = obstacle->st_boundary();
    if (!st_boundary.IsEmpty()) {
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&st_boundary);
    }
  }

  const double min_s_on_st_boundaries = SetSpeedFallbackDistance(path_decision);

  // 3. Create speed limit along path
  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, speed_bounds_config_,
                                        reference_line, path_data);

  SpeedLimit speed_limit;
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
           .ok()) {
    std::string msg("Getting speed limits failed!");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 3. Get path_length as s axis search bound in st graph
  const double path_data_length = path_data.discretized_path().Length();
  const double path_length_by_conf = speed_bounds_config_.total_path_length();

  // 4. Get time duration as t axis search bound in st graph
  const double total_time_by_conf = speed_bounds_config_.total_time();

  // Load generated st graph data back to frame
  StGraphData *st_graph_data = reference_line_info_->mutable_st_graph_data();

  // Add a st_graph debug info and save the pointer to st_graph_data for
  // optimizer logging
  auto *debug = reference_line_info_->mutable_debug();
  STGraphDebug *st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, path_data_length, path_length_by_conf,
                          total_time_by_conf, st_graph_debug);

  // Create and record st_graph debug info
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);

  return Status::OK();
}

void SpeedBoundsDecider::CheckLaneChangeUrgency(Frame *const frame) {
  for (auto &reference_line_info : *frame->mutable_reference_line_info()) {
    // Check if the target lane is blocked or not
    if (reference_line_info.IsChangeLanePath()) {
      is_clear_to_change_lane_ =
          ChangeLaneDecider::IsClearToChangeLane(&reference_line_info);
      continue;
    }
    // If it's not in lane-change scenario or target lane is not blocked, skip
    if (frame->reference_line_info().size() <= 1 || is_clear_to_change_lane_) {
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
    if (reference_line->XYToSL({point.x(), point.y()}, &sl_point) &&
        reference_line->IsOnLane(sl_point)) {
      // Check the distance from ADC to the end point of current routing
      double distance_to_passage_end =
          sl_point.s() - reference_line_info.AdcSlBoundary().end_s();
      // If ADC is still far from the end of routing, no need to stop, skip
      if (distance_to_passage_end >
          speed_bounds_config_.approach_distance_for_lane_change()) {
        continue;
      }
      // In urgent case, set a temporary stop fence and wait to change lane
      // TODO(Jiaxuan Xu): replace the stop fence to more intelligent actions
      const std::string stop_wall_id = "lane_change_stop";
      std::vector<std::string> wait_for_obstacles;
      util::BuildStopDecision(
          stop_wall_id, sl_point.s(),
          speed_bounds_config_.urgent_distance_for_lane_change(),
          StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY, wait_for_obstacles,
          "SpeedBoundsDecider", frame, &reference_line_info);
    }
  }
}

void SpeedBoundsDecider::AddPathEndStop(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  const std::string stop_wall_id = "path_end_stop";
  std::vector<std::string> wait_for_obstacles;
  if (!reference_line_info->path_data().path_label().empty() &&
      reference_line_info->path_data().frenet_frame_path().back().s() -
              reference_line_info->path_data().frenet_frame_path().front().s() <
          FLAGS_short_path_length_threshold) {
    util::BuildStopDecision(
        stop_wall_id,
        reference_line_info->path_data().frenet_frame_path().back().s() - 5.0,
        0.0, StopReasonCode::STOP_REASON_LANE_CHANGE_URGENCY,
        wait_for_obstacles, "SpeedBoundsDecider", frame, reference_line_info);
  }
}

double SpeedBoundsDecider::SetSpeedFallbackDistance(
    PathDecision *const path_decision) {
  // Set min_s_on_st_boundaries to guide speed fallback. Different stop distance
  // is taken when there is an obstacle moving in opposite direction of ADV
  constexpr double kEpsilon = 1.0e-6;
  double min_s_non_reverse = std::numeric_limits<double>::infinity();
  double min_s_reverse = std::numeric_limits<double>::infinity();
  // TODO(Jinyun): tmp workaround for side pass capability because of doomed
  // speed planning failure when side pass creeping
  double side_pass_stop_s = std::numeric_limits<double>::infinity();

  for (auto *obstacle : path_decision->obstacles().Items()) {
    const auto &st_boundary = obstacle->st_boundary();

    if (st_boundary.IsEmpty()) {
      continue;
    }

    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) {
      if (min_s_reverse > lowest_s) {
        min_s_reverse = lowest_s;
      }
    } else if (min_s_non_reverse > lowest_s) {
      min_s_non_reverse = lowest_s;
    }

    if (obstacle->LongitudinalDecision().stop().reason_code() ==
            StopReasonCode::STOP_REASON_SIDEPASS_SAFETY &&
        side_pass_stop_s > lowest_s) {
      side_pass_stop_s = lowest_s;
    }
  }

  min_s_reverse = std::max(min_s_reverse, 0.0);
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);
  side_pass_stop_s = std::max(side_pass_stop_s, 0.0);

  if (!std::isinf(side_pass_stop_s)) {
    return side_pass_stop_s;
  }
  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}

void SpeedBoundsDecider::StopOnSidePass(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  const PathData &path_data = reference_line_info->path_data();
  double stop_s_on_pathdata = 0.0;
  bool set_stop_fence = false;
  const auto &side_pass_info = PlanningContext::Instance()->side_pass_info();
  auto *mutable_side_pass_info =
      PlanningContext::Instance()->mutable_side_pass_info();

  if (path_data.path_label().find("self") != std::string::npos) {
    mutable_side_pass_info->check_clear_flag = false;
    mutable_side_pass_info->change_lane_stop_flag = false;
    mutable_side_pass_info->change_lane_stop_path_point.Clear();
    return;
  }

  if (side_pass_info.change_lane_stop_flag) {
    // Check if stop at last frame stop fence
    ADEBUG << "stop fence is set due to side pass on reverse lane";
    if (CheckADCStop(*reference_line_info,
                     side_pass_info.change_lane_stop_path_point)) {
      ADEBUG << "ADV Stopped due to change lane in side pass";
      if (ChangeLaneDecider::IsClearToChangeLane(reference_line_info)) {
        ADEBUG << "Environment clear for ADC to change lane in side pass";
        mutable_side_pass_info->check_clear_flag = true;
      } else {
        if (CheckSidePassStop(path_data, *reference_line_info,
                              &stop_s_on_pathdata) &&
            BuildSidePassStopFence(
                path_data, stop_s_on_pathdata,
                &(mutable_side_pass_info->change_lane_stop_path_point), frame,
                reference_line_info)) {
          set_stop_fence = true;
        } else {
          set_stop_fence =
              BuildSidePassStopFence(side_pass_info.change_lane_stop_path_point,
                                     frame, reference_line_info);
        }
      }
    } else {
      if (CheckSidePassStop(path_data, *reference_line_info,
                            &stop_s_on_pathdata) &&
          BuildSidePassStopFence(
              path_data, stop_s_on_pathdata,
              &(mutable_side_pass_info->change_lane_stop_path_point), frame,
              reference_line_info)) {
        set_stop_fence = true;
      } else {
        set_stop_fence =
            BuildSidePassStopFence(side_pass_info.change_lane_stop_path_point,
                                   frame, reference_line_info);
      }
    }
  } else {
    if (!side_pass_info.check_clear_flag &&
        CheckSidePassStop(path_data, *reference_line_info,
                          &stop_s_on_pathdata)) {
      set_stop_fence = BuildSidePassStopFence(
          path_data, stop_s_on_pathdata,
          &(mutable_side_pass_info->change_lane_stop_path_point), frame,
          reference_line_info);
    }
    if (side_pass_info.check_clear_flag &&
        CheckClearDone(*reference_line_info,
                       side_pass_info.change_lane_stop_path_point)) {
      mutable_side_pass_info->check_clear_flag = false;
    }
  }
  mutable_side_pass_info->change_lane_stop_flag = set_stop_fence;
}

// @brief Check if necessary to set stop fence used for nonscenario side pass
bool SpeedBoundsDecider::CheckSidePassStop(
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
      stop_s_on_pathdata -= nearby_path.GetNearestPoint(
          stop_fence_pose, stop_s_on_pathdata, &stop_l_on_pathdata);
      return true;
    }
    last_path_point_type = std::get<1>(point_guide);
  }
  return false;
}

// @brief Set stop fence for side pass
bool SpeedBoundsDecider::BuildSidePassStopFence(
    const PathData &path_data, const double stop_s_on_pathdata,
    common::PathPoint *stop_pathpoint, Frame *const frame,
    ReferenceLineInfo *const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!path_data.GetPathPointWithRefS(stop_s_on_pathdata, stop_pathpoint)) {
    AERROR << "Can't get stop point on path data";
    return false;
  }

  return BuildSidePassStopFence(*stop_pathpoint, frame, reference_line_info);
}

bool SpeedBoundsDecider::BuildSidePassStopFence(
    const common::PathPoint &stop_point, Frame *const frame,
    ReferenceLineInfo *const reference_line_info) {
  const std::string stop_wall_id = "Side_Pass_Stop";
  // TODO(Jinyun) load relavent surrounding obstacles
  std::vector<std::string> wait_for_obstacles;

  const auto &nearby_path = reference_line_info->reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  nearby_path.GetNearestPoint({stop_point.x(), stop_point.y()}, &stop_point_s,
                              &stop_point_l);

  // TODO(Jinyun) move to confs
  constexpr double stop_buffer = 0.25;
  util::BuildStopDecision(stop_wall_id, stop_point_s - stop_buffer, 0.0,
                          StopReasonCode::STOP_REASON_SIDEPASS_SAFETY,
                          wait_for_obstacles, "SpeedBoundsDecider", frame,
                          reference_line_info);

  return true;
}

// @brief Check if ADV stop at a stop fence
bool SpeedBoundsDecider::CheckADCStop(
    const ReferenceLineInfo &reference_line_info,
    const common::PathPoint &stop_point) {
  const double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  if (adc_speed > speed_bounds_config_.max_adc_stop_speed()) {
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
      speed_bounds_config_.max_valid_stop_distance()) {
    ADEBUG << "not a valid stop. too far from stop line.";
    return false;
  }

  return true;
}

bool SpeedBoundsDecider::CheckClearDone(
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
  reference_line_info.reference_line().XYToSL({stop_point.x(), stop_point.y()},
                                              &stop_sl_point);
  // use distance to last stop point to determine if needed to check clear
  // again
  if (adc_back_edge_s > stop_sl_point.s()) {
    if (adc_start_l > -lane_right_width || adc_end_l < lane_left_width) {
      return true;
    }
  }
  return false;
}

void SpeedBoundsDecider::RecordSTGraphDebug(
    const StGraphData &st_graph_data, STGraphDebug *st_graph_debug) const {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  for (const auto &boundary : st_graph_data.st_boundaries()) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {
      case STBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case STBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case STBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case STBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case STBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case STBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
    }

    for (const auto &point : boundary->points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto &point : st_graph_data.speed_limit().speed_limit_points()) {
    common::SpeedPoint speed_point;
    speed_point.set_s(point.first);
    speed_point.set_v(point.second);
    st_graph_debug->add_speed_limit()->CopyFrom(speed_point);
  }
}
}  // namespace planning
}  // namespace apollo
