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

#include "modules/planning/scenarios/side_pass/stage_approach_obstacle.h"

#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;

/*
 * @brief: STAGE ApproachObstacle in side_pass scenario
 */
Stage::StageStatus StageApproachObstacle::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  ADEBUG << "SIDEPASS: Approaching obstacle.";
  std::string blocking_obstacle_id = GetContext()->front_blocking_obstacle_id_;
  const SLBoundary& adc_sl_boundary =
      frame->reference_line_info().front().AdcSlBoundary();
  const PathDecision& path_decision =
      frame->reference_line_info().front().path_decision();

  // Locate the front blocking obstacle.
  // If cannot find it, exit stage.
  double obstacle_start_s = -1.0;
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (obstacle->Id() == blocking_obstacle_id) {
      obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();
      break;
    }
  }
  if (obstacle_start_s < 0.0) {
    AWARN << "Front blocking obstacle: " << blocking_obstacle_id
          << " is not found.";
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }
  if ((obstacle_start_s - adc_sl_boundary.end_s()) <
      GetContext()->scenario_config_.min_front_obstacle_distance()) {
    AWARN << "Front blocking obstacle: " << blocking_obstacle_id
          << " moved to be too close or behind ADC.";
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }

  // Create a virtual stop fence as the end-point of this obstacle
  // approaching stage.
  double stop_fence_s =
      obstacle_start_s -
      GetContext()->scenario_config_.stop_fence_distance_to_blocking_obstacle();
  std::string virtual_obstacle_id = blocking_obstacle_id + "_virtual_stop";
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    auto* obstacle = frame->CreateStopObstacle(
        &reference_line_info, virtual_obstacle_id, stop_fence_s);
    if (!obstacle) {
      AERROR << "Failed to create virtual stop obstacle["
             << blocking_obstacle_id << "]";
      next_stage_ = ScenarioConfig::NO_STAGE;
      return Stage::FINISHED;
    }
    Obstacle* stop_wall = reference_line_info.AddObstacle(obstacle);
    if (!stop_wall) {
      AERROR << "Failed to create stop obstacle for: " << blocking_obstacle_id;
      next_stage_ = ScenarioConfig::NO_STAGE;
      return Stage::FINISHED;
    }

    const double stop_distance = 0.2;
    const double stop_s = stop_fence_s - stop_distance;
    const auto& reference_line = reference_line_info.reference_line();
    auto stop_point = reference_line.GetReferencePoint(stop_s);
    double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

    ObjectDecisionType stop;
    auto stop_decision = stop.mutable_stop();
    stop_decision->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
    stop_decision->set_distance_s(-stop_distance);
    stop_decision->set_stop_heading(stop_heading);
    stop_decision->mutable_stop_point()->set_x(stop_point.x());
    stop_decision->mutable_stop_point()->set_y(stop_point.y());
    stop_decision->mutable_stop_point()->set_z(0.0);

    auto* path_decision = reference_line_info.path_decision();
    path_decision->AddLongitudinalDecision("SidePass", stop_wall->Id(), stop);

    break;
  }

  // Do path planning to stop at a proper distance to the blocking obstacle.
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_start_point, frame);
  if (!plan_ok) {
    AERROR << "Stage " << Name() << " error: "
           << "planning on reference line failed.";
    return Stage::ERROR;
  }
  double adc_velocity = frame->vehicle_state().linear_velocity();

  // Check if it still satisfies the SIDE_PASS criterion.
  // If so, update the distance to front blocking obstacle.
  double distance_to_closest_blocking_obstacle = -100.0;
  bool exists_a_blocking_obstacle = false;
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (IsBlockingObstacleToSidePass(
            *frame, obstacle,
            GetContext()->scenario_config_.block_obstacle_min_speed(),
            GetContext()->scenario_config_.min_front_obstacle_distance(),
            GetContext()->scenario_config_.enable_obstacle_blocked_check())) {
      exists_a_blocking_obstacle = true;
      double distance_between_adc_and_obstacle =
          GetDistanceBetweenADCAndObstacle(*frame, obstacle);
      if (distance_to_closest_blocking_obstacle < 0.0 ||
          distance_between_adc_and_obstacle <
              distance_to_closest_blocking_obstacle) {
        distance_to_closest_blocking_obstacle =
            distance_between_adc_and_obstacle;
      }
    }
  }
  if (!exists_a_blocking_obstacle) {
    next_stage_ = ScenarioConfig::NO_STAGE;
    ADEBUG << "There is no blocking obstacle.";
    return Stage::FINISHED;
  }
  if (distance_to_closest_blocking_obstacle < 0.0) {
    AERROR << "Stage " << Name() << " error: "
           << "front obstacle has wrong position.";
    return Stage::ERROR;
  }

  // If ADC stopped at a proper distance from blocking obstacle,
  // switch to the SIDE_PASS_GENERATE_PATH;
  // otherwise, give it more time to finish this stage.
  double max_stop_velocity =
      GetContext()->scenario_config_.approach_obstacle_max_stop_speed();
  double min_stop_obstacle_distance =
      GetContext()->scenario_config_.approach_obstacle_min_stop_distance();
  ADEBUG << "front_obstacle_distance = "
         << distance_to_closest_blocking_obstacle;
  ADEBUG << "adc_velocity = " << adc_velocity;
  if (adc_velocity < max_stop_velocity &&
      distance_to_closest_blocking_obstacle > min_stop_obstacle_distance) {
    next_stage_ = ScenarioConfig::SIDE_PASS_GENERATE_PATH;
    return Stage::FINISHED;
  }
  return Stage::RUNNING;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
