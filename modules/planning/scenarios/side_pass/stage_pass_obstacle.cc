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

#include "modules/planning/scenarios/side_pass/stage_pass_obstacle.h"

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;
using apollo::common::math::Vec2d;

/*
 * @brief:
 * STAGE: StagePassObstacle
 */
Stage::StageStatus StagePassObstacle::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  ADEBUG << "SIDEPASS: Side-passing!";
  const auto& reference_line_info = frame->reference_line_info().front();
  bool update_success = GetContext()->path_data_.UpdateFrenetFramePath(
      &reference_line_info.reference_line());
  if (!update_success) {
    AERROR << "Fail to update path_data.";
    return Stage::ERROR;
  }
  ADEBUG << "Processing StagePassObstacle.";
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

  // TODO(All):
  // Have to use DpPolyPathOptimizer to show in dreamview. Need change to
  // correct name.
  debug_path->set_name("DpPolyPathOptimizer");
  debug_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_start_point, frame);
  if (!plan_ok) {
    AERROR << "Fail to plan on reference line.";
    return Stage::ERROR;
  }

  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const auto& end_point =
      reference_line_info.path_data().discretized_path().back();
  Vec2d last_xy_point(end_point.x(), end_point.y());
  // get s of last point on path
  common::SLPoint sl_point;
  if (!reference_line_info.reference_line().XYToSL(last_xy_point, &sl_point)) {
    AERROR << "Fail to transfer cartesian point to frenet point.";
    return Stage::ERROR;
  }

  double distance_to_path_end =
      sl_point.s() - GetContext()->scenario_config_.side_pass_exit_distance();

  double adc_velocity = frame->vehicle_state().linear_velocity();
  double max_velocity_for_stop =
      GetContext()->scenario_config_.approach_obstacle_max_stop_speed();
  if (adc_velocity < max_velocity_for_stop) {
    GetContext()->pass_obstacle_stuck_cycle_num_ += 1;
  } else {
    GetContext()->pass_obstacle_stuck_cycle_num_ = 0;
  }
  if (adc_sl_boundary.end_s() > distance_to_path_end ||
      GetContext()->pass_obstacle_stuck_cycle_num_ > 60) {
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }
  return Stage::RUNNING;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
