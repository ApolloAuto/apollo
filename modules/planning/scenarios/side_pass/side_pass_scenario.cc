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

namespace apollo {
namespace planning {

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
}  // namespace

void SidePassScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  init_ = true;
}

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
  const SLBoundary& adc_sl_boundary =
      frame->reference_line_info().front().AdcSlBoundary();
  const auto& frenet_frame_path =
      frame->reference_line_info().front().path_data().frenet_frame_path();
  const auto& frenet_frame_point =
      frenet_frame_path.PointAt(frenet_frame_path.NumOfPoints() - 1);
  int adc_start_s = adc_sl_boundary.start_s();
  int path_end_s = frenet_frame_point.s();
  if ((path_end_s - adc_start_s) > 20.0) {
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

}  // namespace planning
}  // namespace apollo
