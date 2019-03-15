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

#include <string>
#include <tuple>
#include <vector>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/change_lane_decider.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/deciders/decider_rule_based_stop.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include "modules/planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
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

  // 1. Rule_based speed planning configurations for different traffic scenarios
  if (FLAGS_enable_nonscenario_side_pass) {
    double stop_s_on_pathdata = 0.0;
    bool set_stop_fence = false;
    const auto &side_pass_info = PlanningContext::side_pass_info();
    auto *mutable_side_pass_info = PlanningContext::mutable_side_pass_info();
    if (side_pass_info.change_lane_stop_flag) {
      // Check if stop at last frame stop fence
      if (CheckADCStop(*reference_line_info,
                       side_pass_info.change_lane_stop_path_point)) {
        if (!ChangeLaneDecider::IsClearToChangeLane(reference_line_info)) {
          if (CheckSidePassStop(path_data, &stop_s_on_pathdata)) {
            set_stop_fence = BuildSidePassStopFence(
                path_data, stop_s_on_pathdata,
                &(mutable_side_pass_info->change_lane_stop_path_point), frame,
                reference_line_info);
          }
        }
      } else {
        if (CheckSidePassStop(path_data, &stop_s_on_pathdata)) {
          set_stop_fence = BuildSidePassStopFence(
              path_data, stop_s_on_pathdata,
              &(mutable_side_pass_info->change_lane_stop_path_point), frame,
              reference_line_info);
        }
      }
    } else {
      if (CheckSidePassStop(path_data, &stop_s_on_pathdata)) {
        set_stop_fence = BuildSidePassStopFence(
            path_data, stop_s_on_pathdata,
            &(mutable_side_pass_info->change_lane_stop_path_point), frame,
            reference_line_info);
      }
    }
    mutable_side_pass_info->change_lane_stop_flag = set_stop_fence;
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
    auto id = obstacle->Id();
    if (!obstacle->st_boundary().IsEmpty()) {
      if (obstacle->st_boundary().boundary_type() ==
          STBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&obstacle->st_boundary());
    }
  }

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

  st_graph_data->LoadData(boundaries, init_point, speed_limit, path_data_length,
                          path_length_by_conf, total_time_by_conf,
                          st_graph_debug);

  // Create and record st_graph debug info
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);

  return Status::OK();
}

// @brief Check if necessary to set stop fence used for nonscenario side pass
bool SpeedBoundsDecider::CheckSidePassStop(const PathData &path_data,
                                           double *stop_s_on_pathdata) {
  const std::vector<std::tuple<double, PathData::PathPointType, double>>
      &path_point_decision_guide = path_data.path_point_decision_guide();
  PathData::PathPointType last_path_point_type =
      PathData::PathPointType::UNKNOWN;
  for (const auto &point_guide : path_point_decision_guide) {
    if (last_path_point_type == PathData::PathPointType::IN_LANE &&
        (std::get<1>(point_guide) == PathData::PathPointType::IN_LANE ||
         std::get<1>(point_guide) ==
             PathData::PathPointType::OUT_ON_REVERSE_LANE)) {
      *stop_s_on_pathdata = std::get<0>(point_guide);
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

  if (!path_data.GetPathPointWithPathS(stop_s_on_pathdata, stop_pathpoint)) {
    return false;
  }

  const std::string stop_wall_id = "side_pass_stop";
  // TODO(Jinyun) load relavent surrounding obstacles
  std::vector<std::string> wait_for_obstacles;

  const auto &nearby_path = reference_line_info->reference_line().map_path();
  double stop_point_s = 0.0;
  double stop_point_l = 0.0;
  nearby_path.GetNearestPoint({stop_pathpoint->x(), stop_pathpoint->y()},
                              &stop_point_s, &stop_point_l);

  DeciderRuleBasedStop::BuildStopDecision(
      stop_wall_id, stop_point_s, 0.0,
      StopReasonCode::STOP_REASON_SIDEPASS_SAFETY, wait_for_obstacles, frame,
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
