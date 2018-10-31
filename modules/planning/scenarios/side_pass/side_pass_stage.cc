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
    AERROR << "Stage " << Name() << " error: "
           << "planning on reference line failed.";
    return Stage::ERROR;
  }
  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  double adc_velocity = frame->vehicle_state().linear_velocity();
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const PathDecision& path_decision = reference_line_info.path_decision();

  double front_obstacle_distance = 1000;
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    bool is_on_road = reference_line_info.reference_line().HasOverlap(
        obstacle->PerceptionBoundingBox());
    if (!is_on_road) {
      continue;
    }

    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    if (obstacle_sl.end_s() <= reference_line_info.AdcSlBoundary().start_s()) {
      continue;
    }

    double distance = obstacle_sl.start_s() - adc_front_edge_s;
    if (distance < front_obstacle_distance) {
      front_obstacle_distance = distance;
    }
  }

  if ((front_obstacle_distance) < 0) {
    AERROR << "Stage " << Name() << " error: "
           << "front obstacle has wrong position.";
    return Stage::ERROR;
  }
  // TODO(all): stage params need to be in config file
  double max_stop_velocity = 1.0e-5;
  double min_stop_obstacle_distance = 4.0;

  if (adc_velocity < max_stop_velocity &&
      front_obstacle_distance > min_stop_obstacle_distance) {
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
    GetContext()->path_data_ = frame->reference_line_info().front().path_data();
    next_stage_ = ScenarioConfig::SIDE_PASS_STOP_ON_WAITPOINT;
    return Stage::FINISHED;
  } else {
    return Stage::ERROR;
  }
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
  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  const PathDecision& path_decision =
      frame->reference_line_info().front().path_decision();
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (obstacle->IsVirtual() &&
        obstacle->PerceptionSLBoundary().start_s() >= adc_front_edge_s) {
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
  const auto& reference_line_info = frame->reference_line_info().front();
  bool update_success = GetContext()->path_data_.UpdateFrenetFramePath(
      &reference_line_info.reference_line());
  if (!update_success) {
    return Stage::ERROR;
  }
  bool trim_success = GetContext()->path_data_.LeftTrimWithRefS(
      planning_start_point.path_point().s());
  if (!trim_success) {
    return Stage::ERROR;
  }

  auto& rfl_info = frame->mutable_reference_line_info()->front();
  *(rfl_info.mutable_path_data()) = GetContext()->path_data_;

  bool plan_ok = PlanningOnReferenceLine(planning_start_point, frame);
  if (!plan_ok) {
    return Stage::ERROR;
  }

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
