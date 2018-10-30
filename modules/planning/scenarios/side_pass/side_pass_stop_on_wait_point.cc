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

#include "modules/planning/scenarios/side_pass/side_pass_stop_on_wait_point.h"

#include <algorithm>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;
using apollo::common::PathPoint;
using apollo::common::math::Vec2d;

constexpr double kExtraMarginforStopOnWaitPointStage = 3.0;

Stage::StageStatus SidePassStopOnWaitPoint::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const PathDecision& path_decision = reference_line_info.path_decision();

  if (GetContext()->path_data_.discretized_path().path_points().empty()) {
    AERROR << "path data is empty.";
    return Stage::ERROR;
  }
  PathPoint first_path_point =
      GetContext()->path_data_.discretized_path().path_points().front();

  PathPoint last_path_point;
  if (!GetMoveForwardLastPathPoint(reference_line, &last_path_point)) {
    AERROR << "Fail to get move forward last path point.";
    return Stage::ERROR;
  }
  double move_forward_distance = last_path_point.s() - first_path_point.s();
  if (!IsFarAwayFromObstacles(reference_line, path_decision.obstacles(),
                              first_path_point, last_path_point)) {
    // wait here, do nothing this cycle.
    return Stage::RUNNING;
  }

  // (1) call proceed with cautious
  constexpr double kSidePassCreepSpeed = 2.33;  // m/s
  auto& rfl_info = frame->mutable_reference_line_info()->front();
  *(rfl_info.mutable_speed_data()) =
      SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(
          move_forward_distance, kSidePassCreepSpeed);

  // (2) combine path and speed.
  *(rfl_info.mutable_path_data()) = GetContext()->path_data_;

  rfl_info.set_trajectory_type(ADCTrajectory::NORMAL);
  DiscretizedTrajectory trajectory;
  if (!rfl_info.CombinePathAndSpeedProfile(
          frame->PlanningStartPoint().relative_time(),
          frame->PlanningStartPoint().path_point().s(), &trajectory)) {
    AERROR << "Fail to aggregate planning trajectory.";
    return Stage::RUNNING;
  }
  rfl_info.SetTrajectory(trajectory);
  rfl_info.SetDrivable(true);

  next_stage_ = ScenarioConfig::SIDE_PASS_DETECT_SAFETY;
  return Stage::FINISHED;
}

bool SidePassStopOnWaitPoint::IsFarAwayFromObstacles(
    const ReferenceLine& reference_line,
    const IndexedList<std::string, Obstacle>& indexed_obstacle_list,
    const PathPoint& first_path_point, const PathPoint& last_path_point) {
  common::SLPoint first_sl_point;
  if (!reference_line.XYToSL(Vec2d(first_path_point.x(), first_path_point.y()),
                             &first_sl_point)) {
    AERROR << "Failed to get the projection from TrajectoryPoint onto "
              "reference_line";
    return false;
  }
  common::SLPoint last_sl_point;
  if (!reference_line.XYToSL(Vec2d(last_path_point.x(), last_path_point.y()),
                             &last_sl_point)) {
    AERROR << "Failed to get the projection from TrajectoryPoint onto "
              "reference_line";
    return false;
  }

  // Go through every obstacle, check if there is any in the no_obs_zone,
  // which will used by the proceed_with_caution movement.
  for (const auto* obstacle : indexed_obstacle_list.Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    // Check the s-direction.
    double obs_start_s = obstacle->PerceptionSLBoundary().start_s();
    double obs_end_s = obstacle->PerceptionSLBoundary().end_s();
    if (obs_end_s < first_sl_point.s() ||
        obs_start_s > last_sl_point.s() + kExtraMarginforStopOnWaitPointStage) {
      continue;
    }
    // Check the l-direction.
    double lane_left_width_at_start_s = 0.0;
    double lane_left_width_at_end_s = 0.0;
    double lane_right_width_at_start_s = 0.0;
    double lane_right_width_at_end_s = 0.0;
    reference_line.GetLaneWidth(obs_start_s, &lane_left_width_at_start_s,
                                &lane_right_width_at_start_s);
    reference_line.GetLaneWidth(obs_end_s, &lane_left_width_at_end_s,
                                &lane_right_width_at_end_s);
    double lane_left_width = std::min(std::abs(lane_left_width_at_start_s),
                                      std::abs(lane_left_width_at_end_s));
    double lane_right_width = std::min(std::abs(lane_right_width_at_start_s),
                                       std::abs(lane_right_width_at_end_s));
    double obs_start_l = obstacle->PerceptionSLBoundary().start_l();
    double obs_end_l = obstacle->PerceptionSLBoundary().end_l();
    if (obs_start_l < lane_left_width && -obs_end_l < lane_right_width) {
      return false;
    }
  }
  return true;
}

bool SidePassStopOnWaitPoint::GetMoveForwardLastPathPoint(
    const ReferenceLine& reference_line,
    common::PathPoint* const last_path_point) {
  int count = 0;
  for (const auto& path_point :
       GetContext()->path_data_.discretized_path().path_points()) {
    // Get the four corner points ABCD of ADC at every path point,
    // and check if that's within the current lane until it reaches
    // out of current lane.
    const auto& vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
    std::vector<Vec2d> ABCDpoints = vehicle_box.GetAllCorners();
    bool is_out_of_curr_lane = false;
    for (size_t i = 0; i < ABCDpoints.size(); i++) {
      // For each corner point, project it onto reference_line
      common::SLPoint curr_point_sl;
      if (!reference_line.XYToSL(ABCDpoints[i], &curr_point_sl)) {
        AERROR << "Failed to get the projection from point onto "
                  "reference_line";
        return false;
      }
      // Get the lane width at the current s indicated by path_point
      double curr_point_left_width = 0.0;
      double curr_point_right_width = 0.0;
      reference_line.GetLaneWidth(curr_point_sl.s(), &curr_point_left_width,
                                  &curr_point_right_width);
      // Check if this corner point is within the lane:
      if (curr_point_sl.l() > std::abs(curr_point_left_width) ||
          curr_point_sl.l() < -std::abs(curr_point_right_width)) {
        is_out_of_curr_lane = true;
        break;
      }
    }
    if (is_out_of_curr_lane) {
      if (count == 0) {
        // The current ADC, without moving at all, is already at least
        // partially out of the current lane.
        return Stage::FINISHED;
      }
      break;
    } else {
      *last_path_point = path_point;
    }
    // check if the ego car on path_point will partially go into the
    // neighbor lane, and retain only those within-lane path-points.
    CHECK_GE(path_point.s(), 0.0);
    ++count;
  }
  return true;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
