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

#include <algorithm>
#include <string>
#include <vector>

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
using apollo::common::VehicleConfigHelper;

/*
 * @brief: STAGE ApproachObstacle in side_pass scenario
 */
Stage::StageStatus StageApproachObstacle::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  std::string blocking_obstacle_id = GetContext()->front_blocking_obstacle_id_;
  const SLBoundary& adc_sl_boundary =
      frame->reference_line_info().front().AdcSlBoundary();
  const PathDecision& path_decision =
      frame->reference_line_info().front().path_decision();

  double obstacle_start_s = -1.0;
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (obstacle->Id() == blocking_obstacle_id) {
      obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();
      break;
    }
  }
  if (obstacle_start_s < 0.0) {
    AWARN << "front blocking obstacle: " << blocking_obstacle_id
          << " is not found";
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }

  if ((obstacle_start_s - adc_sl_boundary.end_s()) <
      GetContext()->scenario_config_.min_front_obstacle_distance()) {
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }

  double kBlockingObstacleDistance = 5.0;
  double stop_fence_s = obstacle_start_s - kBlockingObstacleDistance;
  stop_fence_s = std::max(stop_fence_s, adc_sl_boundary.end_s() + 0.2);
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

  // check the status of side pass scenario
  bool has_blocking_obstacle = false;
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (obstacle->IsVirtual() || !obstacle->IsStatic()) {
      continue;
    }
    CHECK(obstacle->IsStatic());
    if (obstacle->speed() >
        GetContext()->scenario_config_.block_obstacle_min_speed()) {
      continue;
    }
    if (obstacle->PerceptionSLBoundary().start_s() <=
        adc_sl_boundary.end_s()) {  // such vehicles are behind the ego car.
      continue;
    }
    if (obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() +
            GetContext()->scenario_config_.max_front_obstacle_distance()) {
      // vehicles are far away
      continue;
    }

    // check driving_width
    const auto& reference_line =
        frame->reference_line_info().front().reference_line();
    const double driving_width =
        reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());
    const double adc_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width();
    if (driving_width - adc_width - FLAGS_static_decision_nudge_l_buffer >
        GetContext()->scenario_config_.min_l_nudge_buffer()) {
      continue;
    }

    has_blocking_obstacle = true;
    break;
  }

  if (!has_blocking_obstacle) {
    next_stage_ = ScenarioConfig::NO_STAGE;
    return Stage::FINISHED;
  }
  // do path planning
  bool plan_ok = ExecuteTaskOnReferenceLine(planning_start_point, frame);
  if (!plan_ok) {
    AERROR << "Stage " << Name() << " error: "
           << "planning on reference line failed.";
    return Stage::ERROR;
  }

  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  double adc_velocity = frame->vehicle_state().linear_velocity();
  double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

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

  double max_stop_velocity =
      GetContext()->scenario_config_.approach_obstacle_max_stop_speed();
  double min_stop_obstacle_distance =
      GetContext()->scenario_config_.approach_obstacle_min_stop_distance();

  if (adc_velocity < max_stop_velocity &&
      front_obstacle_distance > min_stop_obstacle_distance) {
    next_stage_ = ScenarioConfig::SIDE_PASS_GENERATE_PATH;
    return Stage::FINISHED;
  }

  return Stage::RUNNING;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
