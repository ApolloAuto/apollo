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

#include "modules/planning/scenarios/side_pass/stage_stop_on_wait_point.h"

#include <algorithm>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::TrajectoryPoint;
using apollo::common::PathPoint;
using apollo::common::math::Vec2d;
using apollo::common::VehicleConfigHelper;

constexpr double kSExtraMarginforStopOnWaitPointStage = 3.0;
constexpr double kLExtraMarginforStopOnWaitPointStage = 0.4;

Stage::StageStatus StageStopOnWaitPoint::Process(
    const TrajectoryPoint& planning_start_point, Frame* frame) {
  ADEBUG << "Processing StageStopOnWaitPoint";
  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const PathDecision& path_decision = reference_line_info.path_decision();

  if (GetContext()->path_data_.discretized_path().empty()) {
    AERROR << "path data is empty.";
    return Stage::ERROR;
  }

  if (!GetContext()->path_data_.UpdateFrenetFramePath(&reference_line)) {
    return Stage::ERROR;
  }

  const auto adc_frenet_frame_point_ =
      reference_line.GetFrenetPoint(frame->PlanningStartPoint().path_point());

  if (!GetContext()->path_data_.LeftTrimWithRefS(adc_frenet_frame_point_)) {
    return Stage::ERROR;
  }
  if (GetContext()->path_data_.discretized_path().empty()) {
    AERROR << "path data is empty after trim.";
    return Stage::ERROR;
  }

  for (const auto& p : GetContext()->path_data_.discretized_path()) {
    ADEBUG << p.ShortDebugString();
  }

  // Get the nearest obstacle
  const Obstacle* nearest_obstacle = nullptr;
  if (!GetTheNearestObstacle(reference_line, path_decision.obstacles(),
                             &nearest_obstacle)) {
    AERROR << "Failed while running the function to get nearest obstacle.";
    return Stage::ERROR;
  }

  // If the nearest obstacle, provided it exists, is moving,
  // then quit the side_pass stage.
  if (nearest_obstacle) {
    if (nearest_obstacle->speed() >
        GetContext()->scenario_config_.block_obstacle_min_speed()) {
      ADEBUG << "The nearest obstacle to side-pass is moving.";
      next_stage_ = ScenarioConfig::NO_STAGE;
      return Stage::FINISHED;
    }
  }

  ADEBUG << "Got the nearest obstacle if there is one.";
  // Get the "wait point".
  PathPoint first_path_point =
      GetContext()->path_data_.discretized_path().front();
  PathPoint last_path_point;
  bool should_not_move_at_all = false;
  if (!GetMoveForwardLastPathPoint(reference_line, nearest_obstacle,
                                   &last_path_point, &should_not_move_at_all)) {
    ADEBUG << "Fail to get move forward last path point.";
    return Stage::ERROR;
  }
  if (should_not_move_at_all) {
    ADEBUG << "The ADC is already at a stop point.";
    next_stage_ = ScenarioConfig::SIDE_PASS_DETECT_SAFETY;
    return Stage::FINISHED;  // return FINISHED if it's already at "wait point".
  }

  ADEBUG << "first_path_point: " << first_path_point.ShortDebugString();
  ADEBUG << "last_path_point : " << last_path_point.ShortDebugString();
  double move_forward_distance = last_path_point.s() - first_path_point.s();
  ADEBUG << "move_forward_distance: " << move_forward_distance;

  // Wait until everything is clear.
  if (!IsFarAwayFromObstacles(reference_line, path_decision.obstacles(),
                              first_path_point, last_path_point)) {
    // wait here, do nothing this cycle.
    auto& rfl_info = frame->mutable_reference_line_info()->front();
    *(rfl_info.mutable_path_data()) = GetContext()->path_data_;
    *(rfl_info.mutable_speed_data()) =
        SpeedProfileGenerator::GenerateFallbackSpeedProfile();

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

    ADEBUG << "waiting until obstacles are far away.";
    return Stage::RUNNING;
  }

  // (1) call proceed with cautious
  constexpr double kSidePassCreepSpeed = 2.33;  // m/s
  auto& rfl_info = frame->mutable_reference_line_info()->front();
  *(rfl_info.mutable_speed_data()) =
      SpeedProfileGenerator::GenerateFixedDistanceCreepProfile(
          move_forward_distance, kSidePassCreepSpeed);

  for (const auto& sd : *rfl_info.mutable_speed_data()) {
    ADEBUG << sd.ShortDebugString();
  }

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

  constexpr double kBuffer = 0.3;
  if (move_forward_distance < kBuffer) {
    next_stage_ = ScenarioConfig::SIDE_PASS_DETECT_SAFETY;
  }
  return Stage::FINISHED;
}

bool StageStopOnWaitPoint::IsFarAwayFromObstacles(
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
  // which will be used by the proceed_with_caution movement.
  for (const auto* obstacle : indexed_obstacle_list.Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    // Check the s-direction.
    double obs_start_s = obstacle->PerceptionSLBoundary().start_s();
    double obs_end_s = obstacle->PerceptionSLBoundary().end_s();
    if (obs_end_s < first_sl_point.s() ||
        obs_start_s > last_sl_point.s() +
            kSExtraMarginforStopOnWaitPointStage) {
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

bool StageStopOnWaitPoint::GetTheNearestObstacle(
    const ReferenceLine& reference_line,
    const IndexedList<std::string, Obstacle>& indexed_obstacle_list,
    const Obstacle** nearest_obstacle) {
  // Get the first path point. This can be used later to
  // filter out other obstaces that are behind ADC.
  PathPoint first_path_point =
      GetContext()->path_data_.discretized_path().front();
  common::SLPoint first_sl_point;
  if (!reference_line.XYToSL(Vec2d(first_path_point.x(), first_path_point.y()),
                             &first_sl_point)) {
    AERROR << "Failed to get the projection from TrajectoryPoint onto "
              "reference_line";
    return false;
  }

  // Go through every obstacle.
  bool exist_nearest_obs = false;
  *nearest_obstacle = nullptr;
  double s_of_nearest_obs = 0.0;
  for (const auto* obstacle : indexed_obstacle_list.Items()) {
    ADEBUG << "Looking at Obstacle: " << obstacle->Id();

    if (obstacle->IsVirtual()) {
      continue;
    }

    // Check the s-direction. Rule out those obstacles that are behind ADC.
    double obs_start_s = obstacle->PerceptionSLBoundary().start_s();
    double obs_end_s = obstacle->PerceptionSLBoundary().end_s();
    if (obs_end_s <= first_sl_point.s()) {
      ADEBUG << "Obstacle behind ADC. Rule out.";
      continue;
    }

    // Check the l-direction. Rule out those obstacles that are not
    // blocking our drive-way.
    // TODO(all): should make this a GFLAG because this is also used by
    // side_pass_scenario.cc. They should be consistent.
    constexpr double kLBufferThreshold = 0.3;  // unit: m
    const double driving_width =
        reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());
    const double adc_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width();

    // Refresh the s of the nearest obstacle if needed.
    if (driving_width - adc_width - FLAGS_static_decision_nudge_l_buffer <=
        kLBufferThreshold) {
      ADEBUG << "Obstacle is not completely outside the current lane.";
      if (!exist_nearest_obs) {
        ADEBUG << "Updating the nearest obstacle to: obstacle_"
               << obstacle->Id();
        exist_nearest_obs = true;
        *nearest_obstacle = obstacle;
        s_of_nearest_obs = obs_start_s;
      } else {
        if (obs_start_s < s_of_nearest_obs) {
          ADEBUG << "Updating the nearest obstacle to: obstacle_"
                 << obstacle->Id();
          *nearest_obstacle = obstacle;
          s_of_nearest_obs = obs_start_s;
        }
      }
    }
  }

  return true;
}

bool StageStopOnWaitPoint::GetMoveForwardLastPathPoint(
    const ReferenceLine& reference_line, const Obstacle* nearest_obstacle,
    PathPoint* const last_path_point, bool* should_not_move_at_all) {
  *should_not_move_at_all = false;
  int count = 0;

  bool exist_nearest_obs = (nearest_obstacle != nullptr);
  double s_max = 0.0;
  if (exist_nearest_obs) {
    ADEBUG << "There exists a nearest obstacle.";
    s_max = nearest_obstacle->PerceptionSLBoundary().start_s();
  }

  for (const auto& path_point : GetContext()->path_data_.discretized_path()) {
    // Get the four corner points ABCD of ADC at every path point,
    // and keep checking until it gets out of the current lane or
    // reaches the nearest obstacle (in the same lane) ahead.
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
      if (curr_point_sl.l() > std::abs(curr_point_left_width) -
          kLExtraMarginforStopOnWaitPointStage ||
          curr_point_sl.l() < -std::abs(curr_point_right_width) +
          kLExtraMarginforStopOnWaitPointStage) {
        is_out_of_curr_lane = true;
        break;
      }
      // Check if this corner point is before the nearest obstacle:
      if (exist_nearest_obs && curr_point_sl.s() > s_max) {
        is_out_of_curr_lane = true;
        break;
      }
    }

    if (is_out_of_curr_lane) {
      if (count == 0) {
        // The current ADC, without moving at all, is already at least
        // partially out of the current lane.
        *should_not_move_at_all = true;
        return true;
      }
      break;
    } else {
      *last_path_point = path_point;
    }

    CHECK_GE(path_point.s(), 0.0);
    ++count;
  }
  return true;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
