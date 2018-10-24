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

#include "modules/planning/scenarios/side_pass/side_pass_stage.h"

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

constexpr double kExtraMarginforStopOnWaitPointStage = 3.0;

/*
 * @brief:
 * STAGE: SidePassApproachObstacle
 */
Stage::StageStatus SidePassApproachObstacle::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  bool plan_ok = PlanningOnReferenceLine(planning_start_point, frame);
  if (!plan_ok) {
    return Stage::ERROR;
  }
  if (frame->vehicle_state().linear_velocity() < 1.0e-5) {
    next_stage_ = ScenarioConfig::SIDE_PASS_GENERATE_PATH;
    return Stage::FINISHED;
  }
  return Stage::RUNNING;
}

/*
 * @brief:
 * STAGE: SidePassGeneratePath
 */
Stage::StageStatus SidePassGeneratePath::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  if (PlanningOnReferenceLine(planning_start_point, frame)) {
    next_stage_ = ScenarioConfig::SIDE_PASS_STOP_ON_WAITPOINT;
    return Stage::FINISHED;
  } else {
    return Stage::ERROR;
  }
}

/**
  * @brief:
  * STAGE: SidePassStopOnWaitPoint
  * Notations:
  *
  *    front of car
  * A +----------+ B
  *   |          |
  *   /          / turn with maximum steering angle
  *   |          |
  *   |          |
  *   |          |
  *   |    X     |                                       O
  *   |<-->.<----|-------------------------------------->* (turn center)
  *   |          |   VehicleParam.min_turn_radius()
  *   |          |
  * D +----------+ C
  *    back of car
  *
  */
Stage::StageStatus SidePassStopOnWaitPoint::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const PathDecision& path_decision = reference_line_info.path_decision();

  double move_forward_distance = 5.0;
  CHECK_GT((GetContext()->path_data_.discretized_path().path_points()).size(),
           0);
  common::PathPoint first_path_point =
      (GetContext()->path_data_.discretized_path().path_points())[0];
  common::PathPoint last_path_point;
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
      common::SLPoint curr_point_SL;
      if (!reference_line.XYToSL(ABCDpoints[i], &curr_point_SL)) {
        AERROR << "Failed to get the projection from point onto "
                  "reference_line";
        return Stage::ERROR;
      }
      // Get the lane width at the current s indicated by path_point
      double curr_point_left_width = 0.0;
      double curr_point_right_width = 0.0;
      reference_line.GetLaneWidth(curr_point_SL.s(), &curr_point_left_width,
                                  &curr_point_right_width);
      // Check if this corner point is within the lane:
      if (curr_point_SL.l() > std::abs(curr_point_left_width) ||
          curr_point_SL.l() < -std::abs(curr_point_right_width)) {
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
      last_path_point = path_point;
      move_forward_distance = path_point.s();
    }
    // (1) check if the ego car on path_point will partially go into the
    // neighbor lane, and retain only those within-lane path-points.
    // (2) update move_forward_distance (to be used by proceed_with_caution)
    CHECK_GE(path_point.s(), 0.0);
    CHECK_GE(move_forward_distance, 0.0);
    count++;
  }

  // Based on the first_path_point and last_path_point,
  // get a bounding box on the current lane, which is supposed to have
  // no other obstacle at all before Proceed_with_Caution can be called.
  common::math::Vec2d first_path_point_vec2d(first_path_point.x(),
                                             first_path_point.y());
  common::SLPoint first_path_point_SL;
  if (!reference_line.XYToSL(first_path_point_vec2d, &first_path_point_SL)) {
    AERROR << "Failed to get the projection from TrajectoryPoint onto "
              "reference_line";
    return Stage::ERROR;
  }
  double no_obs_zone_start_s = first_path_point_SL.s();
  common::math::Vec2d last_path_point_vec2d(last_path_point.x(),
                                            last_path_point.y());
  common::SLPoint last_path_point_SL;
  if (!reference_line.XYToSL(last_path_point_vec2d, &last_path_point_SL)) {
    AERROR << "Failed to get the projection from TrajectoryPoint onto "
              "reference_line";
    return Stage::ERROR;
  }
  double no_obs_zone_end_s =
      last_path_point_SL.s() + kExtraMarginforStopOnWaitPointStage;

  bool all_far_away = true;
  // Go through every obstacle, check if there is any in the no_obs_zone,
  // which will used by the proceed_with_caution movement.
  for (const auto* path_obstacle : path_decision.path_obstacles().Items()) {
    if (path_obstacle->obstacle()->IsVirtual()) {
      continue;
    }
    // Check the s-direction.
    double obs_start_s = path_obstacle->PerceptionSLBoundary().start_s();
    double obs_end_s = path_obstacle->PerceptionSLBoundary().end_s();
    if (obs_end_s < no_obs_zone_start_s || obs_start_s > no_obs_zone_end_s) {
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
    double obs_start_l = path_obstacle->PerceptionSLBoundary().start_l();
    double obs_end_l = path_obstacle->PerceptionSLBoundary().end_l();
    if (obs_start_l > lane_left_width || -obs_end_l > lane_right_width) {
      continue;
    }

    all_far_away = false;
  }

  if (!all_far_away) {
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

/*
 * @brief:
 * STAGE: SidePassDetectSafety
 */
Stage::StageStatus SidePassDetectSafety::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  if (PlanningOnReferenceLine(planning_start_point, frame)) {
    return Stage::ERROR;
  }
  bool is_safe = true;
  const PathDecision& path_decision =
      frame->reference_line_info().front().path_decision();
  for (const auto* path_obstacle : path_decision.path_obstacles().Items()) {
    if (path_obstacle->obstacle()->IsVirtual()) {
      is_safe = false;
      break;
    }
  }
  if (is_safe) {
    next_stage_ = ScenarioConfig::SIDE_PASS_PASS_OBSTACLE;
    return Stage::FINISHED;
  }
  return Stage::RUNNING;
}

/*
 * @brief:
 * STAGE: SidePassPassObstacle
 */
Stage::StageStatus SidePassPassObstacle::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  bool plan_ok = PlanningOnReferenceLine(planning_start_point, frame);
  if (!plan_ok) {
    return Stage::ERROR;
  }
  const auto& reference_line_info = frame->reference_line_info().front();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const auto& end_point =
      reference_line_info.path_data().discretized_path().EndPoint();
  Vec2d last_xy_point(end_point.x(), end_point.y());
  // get s of last point on path
  common::SLPoint sl_point;
  if (!reference_line_info.reference_line().XYToSL(last_xy_point, &sl_point)) {
    AERROR << "Fail to transfer cartesian point to frenet point.";
    return Stage::ERROR;
  }

  if (adc_sl_boundary.end_s() > sl_point.s() - 1.0) {
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }
  return Stage::RUNNING;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
