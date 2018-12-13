
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
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
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

using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    SidePassScenario::s_stage_factory_;

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
    msg_ = "side pass obstacle: " + front_blocking_obstacle_id_;
    return (current_scenario.GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE);
  } else if (current_scenario.scenario_type() != ScenarioConfig::LANE_FOLLOW) {
    return false;
  } else {
    bool is_side_pass = IsSidePassScenario(frame);
    if (is_side_pass) {
      msg_ = "side pass obstacle: " + front_blocking_obstacle_id_;
    }
    return is_side_pass;
  }
}

bool SidePassScenario::IsSidePassScenario(const Frame& frame) {
  return (IsFarFromIntersection(frame) && HasBlockingObstacle(frame) &&
          IsFarFromDestination(frame));
}

bool SidePassScenario::IsFarFromDestination(const Frame& frame) {
  if (frame.reference_line_info().size() > 1) {
    return false;
  }
  const auto& reference_line_info = frame.reference_line_info().front();
  const double kClearDistance = 15.0;
  if (reference_line_info.SDistanceToDestination() < kClearDistance) {
    ADEBUG << "too close to destination";
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
  const double kClearDistance = 15.0;  // in meters
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
        ADEBUG << "skip side pass scenario for it is too close to signal "
                  "intersection: "
               << distance;
        return false;
      }
    } else {
      if (distance < kClearDistance) {
        ADEBUG << "too close to overlap_type[" << overlap.first << "]";
        return false;
      }
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
    ADEBUG << "Evaluating Obstacle: " << obstacle->Id();
    if (obstacle->IsVirtual()) {
      ADEBUG << " - It is virtual.";
      continue;
    }

    if (!obstacle->IsStatic() ||
        obstacle->speed() >
            side_pass_context_.scenario_config_.block_obstacle_min_speed()) {
      ADEBUG << " - It is non-static.";
      continue;
    }

    if (obstacle->PerceptionSLBoundary().start_s() <=
        adc_sl_boundary.end_s()) {  // such vehicles are behind the ego car.
      ADEBUG << " - It is behind ADC.";
      continue;
    }

    // check s distance
    constexpr double kAdcDistanceThreshold = 15.0;  // unit: m
    if (obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s() + kAdcDistanceThreshold) {
      // vehicles are far away
      ADEBUG << " - It is too far ahead.";
      continue;
    }

    if (adc_sl_boundary.end_s() +
            side_pass_context_.scenario_config_.min_front_obstacle_distance() >
        obstacle->PerceptionSLBoundary().start_s()) {
      // front obstacle is too close to side pass
      ADEBUG << " - It is too close to side-pass.";
      continue;
    }

    // check driving_width
    const auto& reference_line = reference_line_info.reference_line();
    const double driving_width =
        reference_line.GetDrivingWidth(obstacle->PerceptionSLBoundary());
    const double adc_width =
        VehicleConfigHelper::GetConfig().vehicle_param().width();
    if (driving_width >
        adc_width + FLAGS_static_decision_nudge_l_buffer +
            side_pass_context_.scenario_config_.min_l_nudge_buffer()) {
      ADEBUG << " - It is not blocking our way."
             << " (driving width = " << driving_width
             << ", adc_width = " << adc_width << ")";
      continue;
    }

    bool is_blocked_by_others = false;
    if (side_pass_context_.scenario_config_.enable_obstacle_blocked_check() &&
        !IsParked(reference_line, obstacle)) {
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
      ADEBUG << "IT IS BLOCKING!";
      side_pass_context_.front_blocking_obstacle_id_ = obstacle->Id();
      return true;
    } else {
      ADEBUG << " - It is blocked by others.";
    }
  }
  side_pass_context_.front_blocking_obstacle_id_ = "";
  return false;
}

bool SidePassScenario::IsParked(const ReferenceLine& reference_line,
                                const Obstacle* obstacle) {
  if (!FLAGS_enable_scenario_side_pass_multiple_parked_obstacles) {
    return false;
  }
  double road_left_width = 0.0;
  double road_right_width = 0.0;
  double max_road_right_width = 0.0;
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().start_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = road_right_width;
  reference_line.GetRoadWidth(obstacle->PerceptionSLBoundary().end_s(),
                              &road_left_width, &road_right_width);
  max_road_right_width = std::max(max_road_right_width, road_right_width);
  bool is_at_road_edge = std::abs(obstacle->PerceptionSLBoundary().start_l()) >
                         max_road_right_width - 0.1;

  std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  auto obstacle_box = obstacle->PerceptionBoundingBox();
  HDMapUtil::BaseMapPtr()->GetLanes(
      common::util::MakePointENU(obstacle_box.center().x(),
                                 obstacle_box.center().y(), 0.0),
      std::min(obstacle_box.width(), obstacle_box.length()), &lanes);
  bool is_on_parking_lane = false;
  if (lanes.size() == 1 &&
      lanes.front()->lane().type() == apollo::hdmap::Lane::PARKING) {
    is_on_parking_lane = true;
  }

  bool is_parked = is_on_parking_lane || is_at_road_edge;
  return (is_parked && obstacle->IsStatic());
}

}  // namespace side_pass
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
