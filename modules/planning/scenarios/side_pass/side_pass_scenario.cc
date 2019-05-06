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
#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/scenarios/side_pass/stage_side_pass.h"

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
constexpr double kSidePassMaxSpeed = 5.0;  // (10mph)
constexpr double kSidePassMaxDistance = 10.0;

void SidePassScenario::RegisterStages() {
  s_stage_factory_.Clear();
  s_stage_factory_.Register(
      ScenarioConfig::SIDE_PASS_DEFAULT_STAGE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageSidePass(config);
      });
}

SidePassScenario::SidePassScenario(const ScenarioConfig& config,
                                   const ScenarioContext* scenario_context)
    : Scenario(config, scenario_context) {
  side_pass_context_.scenario_config_.CopyFrom(config.side_pass_config());

  // TODO(all): to be removed when SidePass obstacle decision impl is ready
  side_pass_context_.front_blocking_obstacle_id_ =
      PlanningContext::Instance()
          ->planning_status()
          .side_pass()
          .front_blocking_obstacle_id();
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

bool SidePassScenario::IsTransferable(const Frame& frame,
                                      const ScenarioConfig& config,
                                      const Scenario& current_scenario) {
  // Sanity checks.
  if (frame.reference_line_info().size() > 1) {
    return false;
  }

  if (config.stage_type(0) == ScenarioConfig::SIDE_PASS_DEFAULT_STAGE) {
    return false;
    return IsUnifiedTransferable(frame, config, current_scenario);
  }

  std::string front_blocking_obstacle_id = PlanningContext::Instance()
                                               ->planning_status()
                                               .side_pass()
                                               .front_blocking_obstacle_id();

  if (current_scenario.scenario_type() == ScenarioConfig::SIDE_PASS) {
    // Check if the blocking obstacle is still static.
    // If not, then switch to LANE_FOLLOW.
    const auto front_blocking_obstacle =
        const_cast<Frame&>(frame).Find(front_blocking_obstacle_id);
    if (!front_blocking_obstacle) {
      ADEBUG << "Obstacle " << front_blocking_obstacle_id
             << " not exist any more. Change scenario to default scenario.";
      return false;
    }
    const auto& reference_line_info = frame.reference_line_info().front();
    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double distance =
        front_blocking_obstacle->PerceptionSLBoundary().start_s() -
        adc_front_edge_s;

    if (!front_blocking_obstacle->IsStatic() ||
        distance > kSidePassMaxDistance) {
      ADEBUG << "Obstacle " << front_blocking_obstacle_id
             << " starts to move. Change scenario to default scenario.";
      return false;
    }
    // msg_ = "side pass obstacle: " + front_blocking_obstacle_id;
    return (current_scenario.GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE);
  } else if (current_scenario.scenario_type() != ScenarioConfig::LANE_FOLLOW) {
    // If in some other special scenario, then don't try to switch
    // to SIDE_PASS scenario.
    return false;
  } else {
    // If originally in LANE_FOLLOW, then decide whether we should
    // switch to SIDE_PASS scenario.
    ADEBUG << "Checking if it's needed to switch from LANE_FOLLOW to "
              "SIDE_PASS: ";
    bool is_side_pass = IsSidePassScenario(frame, config);
    if (is_side_pass) {
      // msg_ = "side pass obstacle: " + front_blocking_obstacle_id;
    } else {
    }
    return is_side_pass;
  }
}

bool SidePassScenario::IsUnifiedTransferable(const Frame& frame,
                                             const ScenarioConfig& config,
                                             const Scenario& current_scenario) {
  if (current_scenario.scenario_type() == ScenarioConfig::SIDE_PASS) {
    // Check side-pass exiting conditions.
    // ADEBUG << "Checking if it's needed to exit SIDE_PASS:";
    // ADEBUG << "Able to use self-lane counter = "
    //        << PlanningContext::Instance()->able_to_use_self_lane_counter();
    // return PlanningContext::Instance()->able_to_use_self_lane_counter() < 3;
    return true;
  } else if (current_scenario.scenario_type() != ScenarioConfig::LANE_FOLLOW) {
    // If in some other scenario, then don't try to switch to SIDE_PASS.
    ADEBUG << "Currently in some other scenario.";
    return false;
  } else {
    // If originally in LANE_FOLLOW, then decide whether we should
    // switch to SIDE_PASS scenario.
    ADEBUG << "Checking if it's needed to switch from LANE_FOLLOW to "
              "SIDE_PASS: ";
    bool is_side_pass = IsUnifiedSidePassScenario(frame, config);
    if (is_side_pass) {
      ADEBUG << "   YES!";
    } else {
      ADEBUG << "   NO!";
    }
    return is_side_pass &&
           PlanningContext::Instance()->front_static_obstacle_cycle_counter() >=
               1;
  }
}

bool SidePassScenario::IsSidePassScenario(const Frame& frame,
                                          const ScenarioConfig& config) {
  return (IsFarFromDestination(frame) && IsFarFromIntersection(frame) &&
          HasBlockingObstacle(frame, config));
}

bool SidePassScenario::IsUnifiedSidePassScenario(const Frame& frame,
                                                 const ScenarioConfig& config) {
  return HasSingleReferenceLine(frame) && IsFarFromDestination(frame) &&
         IsBlockingObstacleWithinDestination(frame) &&
         IsFarFromIntersection(frame) && IsWithinSidePassingSpeedADC(frame) &&
         IsSidePassableObstacle(
             frame, frame.reference_line_info().front(),
             frame.reference_line_info().front().GetBlockingObstacleId());
}

bool SidePassScenario::HasSingleReferenceLine(const Frame& frame) {
  return frame.reference_line_info().size() <= 1;
}

bool SidePassScenario::IsBlockingObstacleWithinDestination(const Frame& frame) {
  const auto& reference_line_info = frame.reference_line_info().front();
  std::string blocking_obstacle_id =
      reference_line_info.GetBlockingObstacleId();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double adc_frenet_s =
      reference_line_info.reference_line()
          .GetFrenetPoint(frame.PlanningStartPoint().path_point())
          .s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_frenet_s;
  ADEBUG << "Destination is within: "
         << reference_line_info.SDistanceToDestination();
  if (blocking_obstacle_s - adc_frenet_s >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
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
    if (overlap.first != ReferenceLineInfo::CLEAR_AREA &&
        overlap.first != ReferenceLineInfo::CROSSWALK &&
        // TODO(yifei) temporarily enable side pass at pnc junction without
        // stop sign and traffic light.
        // overlap.first != ReferenceLineInfo::PNC_JUNCTION &&
        overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }

    auto distance = overlap.second.start_s - adc_sl_boundary.end_s();
    if (overlap.first == ReferenceLineInfo::SIGNAL) {
      if (distance < FLAGS_side_pass_min_signal_intersection_distance) {
        ADEBUG << "Too close to signal intersection (" << distance
               << "m); don't SIDE_PASS.";
        return false;
      }
    } else {
      if (distance < kClearDistance) {
        ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
               << distance << "m); don't SIDE_PASS";
        return false;
      }
    }
  }
  return true;
}

bool SidePassScenario::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < kSidePassMaxSpeed;
}

// TODO(jiacheng): implement this to replace HasBlockingObstacle.
bool SidePassScenario::IsSidePassableObstacle(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::string& blocking_obstacle_id) {
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Doesn't exist such blocking obstalce.";
    return true;
  }
  return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}

bool SidePassScenario::HasBlockingObstacle(const Frame& frame,
                                           const ScenarioConfig& config) {
  // Sanity checks.
  if (frame.reference_line_info().size() > 1) {
    return false;
  }

  if (!config.has_side_pass_config()) {
    AERROR << "miss scenario specific config";
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
    if (IsBlockingObstacleToSidePass(
            frame, obstacle,
            config.side_pass_config().block_obstacle_min_speed(),
            config.side_pass_config().min_front_obstacle_distance(),
            config.side_pass_config().enable_obstacle_blocked_check())) {
      exists_a_blocking_obstacle = true;
      double distance_between_adc_and_obstacle =
          GetDistanceBetweenADCAndObstacle(frame, obstacle);
      if (distance_to_closest_blocking_obstacle < 0.0 ||
          distance_between_adc_and_obstacle <
              distance_to_closest_blocking_obstacle) {
        // Only record the closest front blocking obstacle.
        distance_to_closest_blocking_obstacle =
            distance_between_adc_and_obstacle;
        // TODO(all): to be removed
        //            when SidePass obstacle decision impl is ready
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->mutable_side_pass()
            ->set_front_blocking_obstacle_id(obstacle->Id());
      }
    }
  }
  if (exists_a_blocking_obstacle) {
    return true;
  } else {
    // TODO(all): to be removed when SidePass obstacle decision impl is ready
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->mutable_side_pass()
        ->clear_front_blocking_obstacle_id();
    return false;
  }
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
