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

#include "modules/planning/scenarios/side_pass/side_pass_scenario.h"

#include <fstream>
#include <limits>
#include <utility>
#include <algorithm>

#include "cyber/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/toolkits/optimizers/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/path_decider/path_decider.h"
#include "modules/planning/toolkits/optimizers/poly_st_speed/poly_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/toolkits/optimizers/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/toolkits/optimizers/speed_decider/speed_decider.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using common::ErrorCode;
using common::SLPoint;
using common::SpeedPoint;
using common::TrajectoryPoint;
using common::math::Vec2d;
using common::time::Clock;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
constexpr double kExtraMarginforStopOnWaitPointStage = 3.0;
}  // namespace

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    SidePassScenario::s_stage_factory_;

void SidePassScenario::RegisterStages() {
  s_stage_factory_.Clear();
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_APPROACH_OBSTACLE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new SidePassApproachObstacle(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_DETECT_SAFETY,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new SidePassDetectSafety(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_GENERATE_PATH,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new SidePassGeneratePath(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_STOP_ON_WAITPOINT,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new SidePassStopOnWaitPoint(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_PASS_OBSTACLE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new SidePassPassObstacle(config);
      });
}

std::unique_ptr<Stage> SidePassScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config);
  ptr->SetContext(&side_pass_context_);
  return ptr;
}

bool SidePassScenario::IsTransferable(const Scenario& current_scenario,
                                      const common::TrajectoryPoint& ego_point,
                                      const Frame& frame) const {
  if (frame.reference_line_info().size() > 1) {
    return false;
  }
  if (current_scenario.scenario_type() == ScenarioConfig::SIDE_PASS) {
    return (current_scenario.GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE);
  } else if (current_scenario.scenario_type() != ScenarioConfig::LANE_FOLLOW) {
    return false;
  } else {
    return IsSidePassScenario(ego_point, frame);
  }
}

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
  bool all_far_away = true;
  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  const ReferenceLine &reference_line = reference_line_info.reference_line();
  const PathDecision& path_decision = reference_line_info.path_decision();

  double move_forward_distance = 5.0;
  CHECK_GT
      ((GetContext()->path_data_.discretized_path().path_points()).size(), 0);
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
    for (size_t i = 0; i < ABCDpoints.size(); i ++) {
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
      reference_line.GetLaneWidth(curr_point_SL.s(),
                                  &curr_point_left_width,
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
  double no_obs_zone_end_s = last_path_point_SL.s() +
                             kExtraMarginforStopOnWaitPointStage;

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

  // TODO(All):
  // (1) call proceed with cautious
  // (2) combine path and speed.

  next_stage_ = ScenarioConfig::SIDE_PASS_DETECT_SAFETY;
  return Stage::FINISHED;
}

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
  SLPoint sl_point;
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

bool SidePassScenario::IsSidePassScenario(
    const common::TrajectoryPoint& planning_start_point,
    const Frame& frame) const {
  const SLBoundary& adc_sl_boundary =
      frame.reference_line_info().front().AdcSlBoundary();
  const PathDecision& path_decision =
      frame.reference_line_info().front().path_decision();
  // TODO(lianglia-apollo)

  return HasBlockingObstacle(adc_sl_boundary, path_decision);
}

bool SidePassScenario::IsFarFromIntersection(const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    return false;
  }
  const SLBoundary& adc_sl_boundary =
      frame.reference_line_info().front().AdcSlBoundary();
  const auto& first_encounters =
      frame.reference_line_info().front().FirstEncounteredOverlaps();
  const double kClearDistance = 15.0;  // in meters
  for (const auto& encounter : first_encounters) {
    if (encounter.first != ReferenceLineInfo::SIGNAL ||
        encounter.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }
    if (encounter.second.start_s - adc_sl_boundary.end_s() < kClearDistance) {
      return false;
    }
  }
  return true;
}

bool SidePassScenario::HasBlockingObstacle(
    const SLBoundary& adc_sl_boundary,
    const PathDecision& path_decision) const {
  // a blocking obstacle is an obstacle blocks the road when it is not blocked
  // (by other obstacles or traffic rules)
  for (const auto* path_obstacle : path_decision.path_obstacles().Items()) {
    if (path_obstacle->obstacle()->IsVirtual() ||
        !path_obstacle->obstacle()->IsStatic()) {
      continue;
    }
    CHECK(path_obstacle->obstacle()->IsStatic());

    if (path_obstacle->PerceptionSLBoundary().start_s() <=
        adc_sl_boundary.end_s()) {  // such vehicles are behind the adc.
      continue;
    }
    constexpr double kAdcDistanceThreshold = 15.0;  // unit: m
    if (path_obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() +
            kAdcDistanceThreshold) {  // vehicles are far away
      continue;
    }
    if (path_obstacle->PerceptionSLBoundary().start_l() > 1.0 ||
        path_obstacle->PerceptionSLBoundary().end_l() < -1.0) {
      continue;
    }

    bool is_blocked_by_others = false;
    for (const auto* other_obstacle : path_decision.path_obstacles().Items()) {
      if (other_obstacle->Id() == path_obstacle->Id()) {
        continue;
      }
      if (other_obstacle->PerceptionSLBoundary().start_l() >
              path_obstacle->PerceptionSLBoundary().end_l() ||
          other_obstacle->PerceptionSLBoundary().end_l() <
              path_obstacle->PerceptionSLBoundary().start_l()) {
        // not blocking the backside vehicle
        continue;
      }

      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
                       path_obstacle->PerceptionSLBoundary().end_s();
      if (delta_s < 0.0 || delta_s > kAdcDistanceThreshold) {
        continue;
      } else {
        // TODO(All): fixed the segmentation bug for large vehicles, otherwise
        // the follow line will be problematic.
        // is_blocked_by_others = true; break;
      }
    }
    if (!is_blocked_by_others) {
      return true;
    }
  }
  return false;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
