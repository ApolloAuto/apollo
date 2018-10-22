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

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;

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

/*
 * @brief:
 * STAGE: SidePassStopOnWaitPoint
 */
Stage::StageStatus SidePassStopOnWaitPoint::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  bool all_far_away = false;
  const PathDecision& path_decision =
      frame->reference_line_info().front().path_decision();
  for (const auto* path_obstacle : path_decision.path_obstacles().Items()) {
    if (path_obstacle->obstacle()->IsVirtual()) {
      continue;
    }
    // TODO(All): check all ST boundaries are far away.
  }
  if (!all_far_away) {
    // wait here, do nothing this cycle.
    return Stage::RUNNING;
  }
  double move_forward_distance = 5.0;
  for (const auto& path_point :
       GetContext()->path_data_.discretized_path().path_points()) {
    // TODO(All):
    // (1) check if the ego car on path_point will partially go into the
    // neighbor
    // lane.
    // (2) update move_forward_distance
    CHECK_GE(path_point.s(), 0.0);
    CHECK_GE(move_forward_distance, 0.0);
  }
  // TODO(All):
  // (1) call proceed with cautious
  // (2) combine path and speed.

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
