
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

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/scenarios/side_pass/stage_approach_obstacle.h"
#include "modules/planning/scenarios/side_pass/stage_backup.h"
#include "modules/planning/scenarios/side_pass/stage_detect_safety.h"
#include "modules/planning/scenarios/side_pass/stage_generate_path.h"
#include "modules/planning/scenarios/side_pass/stage_pass_obstacle.h"
#include "modules/planning/scenarios/side_pass/stage_stop_on_wait_point.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace side_pass {

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    SidePassScenario::s_stage_factory_;

// The clearance distance from intersection/destination.
// If ADC is too close, then do not enter SIDE_PASS.
constexpr double kClearDistance = 15.0;

void SidePassScenario::RegisterStages() {
  s_stage_factory_.Clear();
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_APPROACH_OBSTACLE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageApproachObstacle(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_DETECT_SAFETY,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageDetectSafety(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_GENERATE_PATH,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageGeneratePath(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_STOP_ON_WAITPOINT,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageStopOnWaitPoint(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_PASS_OBSTACLE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StagePassObstacle(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_BACKUP,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageBackup(config);
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
  // Sanity checks.
  if (frame.reference_line_info().size() > 1) {
    return false;
  }
  if (current_scenario.scenario_type() == ScenarioConfig::SIDE_PASS) {
    // Check if the blocking obstacle is still static.
    // If not, then switch to LANE_FOLLOW.
    const auto front_blocking_obstacle =
        const_cast<Frame&>(frame).Find(front_blocking_obstacle_id_);
    if (front_blocking_obstacle != nullptr &&
        !front_blocking_obstacle->IsStatic()) {
      AWARN << "Obstacle " << front_blocking_obstacle_id_
            << " starts to move. Change scenario from SIDE_PASS"
               " back to default scenario.";
      return false;
    }
    msg_ = "side pass obstacle: " + front_blocking_obstacle_id_;
    return (current_scenario.GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE);
  } else if (current_scenario.scenario_type() != ScenarioConfig::LANE_FOLLOW) {
    // If in some other special scenario, then don't try to switch
    // to SIDE_PASS scenario.
    return false;
  } else {
    // If originally in LANE_FOLLOW, then decide whether we shoudl
    // switch to SIDE_PASS scenario.
    bool is_side_pass = IsSidePassScenario(frame);
    if (is_side_pass) {
      msg_ = "side pass obstacle: " + front_blocking_obstacle_id_;
    }
    return is_side_pass;
  }
}

bool SidePassScenario::IsSidePassScenario(const Frame& frame) {
  return (IsFarFromDestination(frame) &&
          IsFarFromIntersection(frame) &&
          HasBlockingObstacle(frame));
}

bool SidePassScenario::IsFarFromDestination(const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    return false;
  }
  const auto& reference_line_info = frame.reference_line_info().front();
  if (reference_line_info.SDistanceToDestination() < kClearDistance) {
    ADEBUG << "Too close to destination; don't SIDE_PASS";
    return false;
  }
  return true;
}

bool SidePassScenario::IsFarFromIntersection(const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    return false;
  }
  const SLBoundary& adc_sl_boundary =
      frame.reference_line_info().front().AdcSlBoundary();
  const auto& first_encountered_overlaps =
      frame.reference_line_info().front().FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << "AdcSL: " << adc_sl_boundary.ShortDebugString();
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();
    if (overlap.first != ReferenceLineInfo::CROSSWALK &&
        overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }
    auto distance = overlap.second.start_s - adc_sl_boundary.end_s();
    if (overlap.first == ReferenceLineInfo::SIGNAL) {
      if (distance < FLAGS_side_pass_min_signal_intersection_distance) {
        ADEBUG << "Too close to signal intersection ("
               << distance << "m); don't SIDE_PASS.";
        return false;
      }
    } else {
      if (distance < kClearDistance) {
        ADEBUG << "Too close to overlap_type[" << overlap.first
               << "] (" << distance << "m); don't SIDE_PASS";
        return false;
      }
    }
  }
  return true;
}

bool SidePassScenario::HasBlockingObstacle(const Frame& frame) {
  // Sanity checks.
  front_blocking_obstacle_id_ = "";
  if (frame.reference_line_info().size() > 1) {
    return false;
  }

  const auto& reference_line_info = frame.reference_line_info().front();
  const PathDecision& path_decision = reference_line_info.path_decision();
  double distance_to_closest_blocking_obstacle = -100.0;
  bool exists_a_blocking_obstacle = false;
  // A blocking obstacle is an obstacle blocks the road when it is not blocked
  // by other obstacles or traffic rules.
  // Loop through every obstacle to locate the closest blocking one, if there
  // exists such an obstacle.
  for (const auto* obstacle : path_decision.obstacles().Items()) {
    const auto& side_pass_config = side_pass_context_.scenario_config_;
    if (IsBlockingObstacleToSidePass(
            frame, obstacle,
            side_pass_config.block_obstacle_min_speed(),
            side_pass_config.min_front_obstacle_distance(),
            side_pass_config.enable_obstacle_blocked_check())) {
      exists_a_blocking_obstacle = true;
      double distance_between_adc_and_obstacle =
          GetDistanceBetweenADCAndObstacle(frame, obstacle);
      if (distance_to_closest_blocking_obstacle < 0.0 ||
          distance_between_adc_and_obstacle <
              distance_to_closest_blocking_obstacle) {
        // Only record the closest front blocking obstacle.
        distance_to_closest_blocking_obstacle =
            distance_between_adc_and_obstacle;
        front_blocking_obstacle_id_ = obstacle->Id() + "_0";
        side_pass_context_.front_blocking_obstacle_id_ = obstacle->Id();
      }
    }
  }
  if (exists_a_blocking_obstacle) {
    return true;
  } else {
    side_pass_context_.front_blocking_obstacle_id_ = "";
    return false;
  }
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
