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

#include "modules/planning/scenarios/side_pass/stage_detect_safety.h"

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;

/*
 * @brief:
 * STAGE: SidePassDetectSafety
 */
Stage::StageStatus StageDetectSafety::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  ADEBUG << "SIDEPASS: Detecting if it's safe to side-pass.";
  const auto& reference_line_info = frame->reference_line_info().front();
  bool update_success = GetContext()->path_data_.UpdateFrenetFramePath(
      &reference_line_info.reference_line());
  if (!update_success) {
    AERROR << "Fail to update path_data.";
    return Stage::ERROR;
  }

  const auto adc_frenet_frame_point_ =
      reference_line_info.reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());

  bool trim_success =
      GetContext()->path_data_.LeftTrimWithRefS(adc_frenet_frame_point_);
  if (!trim_success) {
    AERROR << "Fail to trim path_data. adc_frenet_frame_point: "
           << adc_frenet_frame_point_.ShortDebugString();
    return Stage::ERROR;
  }

  auto& rfl_info = frame->mutable_reference_line_info()->front();
  *(rfl_info.mutable_path_data()) = GetContext()->path_data_;

  const auto& path_points = rfl_info.path_data().discretized_path();
  auto* debug_path =
      rfl_info.mutable_debug()->mutable_planning_data()->add_path();

  debug_path->set_name("DpPolyPathOptimizer");
  debug_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});

  if (!ExecuteTaskOnReferenceLine(planning_start_point, frame)) {
    return Stage::ERROR;
  }
  bool is_safe = true;
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  const PathDecision& path_decision =
      frame->reference_line_info().front().path_decision();
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    // TODO(All): check according to neighbor lane.
    if (obstacle->IsVirtual() && obstacle->Id().substr(0, 3) == "SP_" &&
        obstacle->PerceptionSLBoundary().start_s() >= adc_front_edge_s) {
      is_safe = false;
      break;
    }
  }
  if (is_safe) {
    GetContext()->pass_obstacle_stuck_cycle_num_ = 0;
    next_stage_ = ScenarioConfig::SIDE_PASS_PASS_OBSTACLE;
    return Stage::FINISHED;
  }
  return Stage::RUNNING;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
