
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

#include <algorithm>
#include <limits>
#include <utility>

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/scenarios/side_pass/side_pass_stage.h"
#include "modules/planning/scenarios/side_pass/side_pass_stop_on_wait_point.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

using apollo::common::VehicleConfigHelper;

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
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_BACKUP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new SidePassBackup(config);
      });
}

SidePassScenario::SidePassScenario(const ScenarioConfig& config,
                                   const ScenarioContext* scenario_context)
    : Scenario(config, scenario_context) {
  side_pass_context_.scenario_config_.CopyFrom(config.side_pass_config());
}

std::unique_ptr<Stage> SidePassScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config);
  if (ptr == nullptr) {
    AERROR << "Failed to create stage for config: "
           << stage_config.DebugString();
    return nullptr;
  }
  ptr->SetContext(&side_pass_context_);
  return ptr;
}

bool SidePassScenario::IsTransferable(const Scenario& current_scenario,
                                      const common::TrajectoryPoint& ego_point,
                                      const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    return false;
  }
  if (current_scenario.scenario_type() == ScenarioConfig::SIDE_PASS) {
    const auto front_blocking_obstacle =
        const_cast<Frame&>(frame).Find(front_blocking_obstacle_id_);
    if (front_blocking_obstacle != nullptr &&
        !front_blocking_obstacle->IsStatic()) {
      AWARN << "obstacle: " << front_blocking_obstacle_id_
            << " starts to move and scenario changes SIDE_PASS back to default "
               "scenario.";
      return false;
    }
    return (current_scenario.GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE);
  } else if (current_scenario.scenario_type() != ScenarioConfig::LANE_FOLLOW) {
    return false;
  } else {
    return IsSidePassScenario(frame);
  }
}

bool SidePassScenario::IsSidePassScenario(const Frame& frame) {
  return (IsFarFromIntersection(frame) && HasBlockingObstacle(frame));
}

bool SidePassScenario::IsFarFromIntersection(const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    return false;
  }
  const SLBoundary& adc_sl_boundary =
      frame.reference_line_info().front().AdcSlBoundary();
  const auto& first_encountered_overlaps =
      frame.reference_line_info().front().FirstEncounteredOverlaps();
  const double kClearDistance = 15.0;  // in meters
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << "AdcSL: " << adc_sl_boundary.ShortDebugString();
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();
    if (overlap.first != ReferenceLineInfo::CROSSWALK &&
        overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }
    if (overlap.second.start_s - adc_sl_boundary.end_s() < kClearDistance) {
      ADEBUG << "too close to overlap_type[" << overlap.first << "]";
      return false;
    }
  }
  return true;
}

bool SidePassScenario::HasBlockingObstacle(const Frame& frame) {
  // possible state change: default scenario => side_pass scenario
  front_blocking_obstacle_id_ = "";

  if (frame.reference_line_info().size() > 1) {
    return false;
  }

  const auto& reference_line_info = frame.reference_line_info().front();
  const SLBoundary& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const PathDecision& path_decision = reference_line_info.path_decision();

  // a blocking obstacle is an obstacle blocks the road when it is not blocked
  // (by other obstacles or traffic rules)
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    if (obstacle->IsVirtual() || !obstacle->IsStatic()) {
      continue;
    }

    if (obstacle->speed() >
        side_pass_context_.scenario_config_.block_obstacle_min_speed()) {
      continue;
    }

    if (obstacle->PerceptionSLBoundary().start_s() <=
        adc_sl_boundary.end_s()) {  // such vehicles are behind the ego car.
      continue;
    }

    // check s distance
    constexpr double kAdcDistanceThreshold = 15.0;  // unit: m
    if (obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() + kAdcDistanceThreshold) {
      // vehicles are far away
      continue;
    }

    if (adc_sl_boundary.end_s() + FLAGS_side_pass_min_front_obstacle_distance >
        obstacle->PerceptionSLBoundary().start_s()) {
      // front obstacle is too close to side pass
      continue;
    }

    // check driving_width
    constexpr double kLBufferThreshold = 0.3;  // unit: m
    const auto& reference_line = reference_line_info.reference_line();
    const double driving_width =
        reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());
    const double adc_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width();
    if (driving_width - adc_width - FLAGS_static_decision_nudge_l_buffer >
        kLBufferThreshold) {
      continue;
    }

    bool is_blocked_by_others = false;
    if (side_pass_context_.scenario_config_.enable_obstacle_blocked_check()) {
      for (const auto* other_obstacle : path_decision.obstacles().Items()) {
        if (other_obstacle->Id() == obstacle->Id()) {
          continue;
        }
        if (other_obstacle->PerceptionSLBoundary().start_l() >
                obstacle->PerceptionSLBoundary().end_l() ||
            other_obstacle->PerceptionSLBoundary().end_l() <
                obstacle->PerceptionSLBoundary().start_l()) {
          // not blocking the backside vehicle
          continue;
        }

        double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
            obstacle->PerceptionSLBoundary().end_s();
        if (delta_s < 0.0 || delta_s > kAdcDistanceThreshold) {
          continue;
        } else {
          // TODO(All): fixed the segmentation bug for large vehicles, otherwise
          // the follow line will be problematic.
          is_blocked_by_others = true;
          break;
        }
      }
    }

    if (!is_blocked_by_others) {
      // static obstacle id doesn't contain prediction trajectory suffix.
      front_blocking_obstacle_id_ = obstacle->Id() + "_0";
      ADEBUG << "Obstacle: " << obstacle->Id() << " is blocking.";
      return true;
    }
  }
  return false;
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
