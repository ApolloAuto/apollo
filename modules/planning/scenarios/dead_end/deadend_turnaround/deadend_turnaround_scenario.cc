/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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
#include "modules/planning/scenarios/dead_end/deadend_turnaround/deadend_turnaround_scenario.h"
#include "modules/planning/scenarios/dead_end/deadend_turnaround/stage_approaching_turning_point.h"
#include "modules/planning/scenarios/dead_end/deadend_turnaround/stage_turning.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace deadend_turnaround {

using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;
using apollo::common::PointENU;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::routing::RoutingRequest;
using apollo::common::math::Polygon2d;
using apollo::common::math::LineSegment2d;

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    DeadEndTurnAroundScenario::s_stage_factory_;

void DeadEndTurnAroundScenario::Init() {
  if (init_) {
    return;
  }
  Scenario::Init();
  if (!GetScenarioConfig()) {
    return;
  }

  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
}

void DeadEndTurnAroundScenario::RegisterStages() {
  if (s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::DEADEND_TURNAROUND_APPROACHING_TURNING_POINT,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageApproachingTurningPoint(config, injector);
      });
  s_stage_factory_.Register(
      ScenarioConfig::DEADEND_TURNAROUND_TURNING,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StageTurning(config, injector);
      });
}

std::unique_ptr<Stage> DeadEndTurnAroundScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config, injector);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

bool DeadEndTurnAroundScenario::GetScenarioConfig() {
  if (!config_.has_deadend_turnaround_config()) {
    return false;
  }
  context_.scenario_config.CopyFrom(config_.deadend_turnaround_config());
  return true;
}

bool DeadEndTurnAroundScenario::IsTransferable(
  const Frame& frame,
  const PointENU& dead_end_point,
  const double dead_end_start_range) {
  std::string target_dead_end_id;
  // vaild check
  const hdmap::HDMap* base_map_ptr = hdmap::HDMapUtil::BaseMapPtr();
  std::vector<JunctionInfoConstPtr> junctions;
  JunctionInfoConstPtr junction;
  if (base_map_ptr->GetJunctions(dead_end_point, 1.0, &junctions) != 0) {
    ADEBUG << "Fail to get junctions from base_map.";
    return false;
  }
  if (junctions.size() <= 0) {
    ADEBUG << "No junction from map";
    return false;
  }
  if (!SelectTargetDeadEndJunction(&junctions, dead_end_point, &junction)) {
    ADEBUG << "Target Dead End not found";
    return false;
  }
  target_dead_end_id = junction->id().id();
  const auto& vehicle_state = frame.vehicle_state();
  const auto& nearby_path =
      frame.reference_line_info().front().reference_line().map_path();
  if (!CheckDistanceToDeadEnd(vehicle_state,
                              nearby_path,
                              dead_end_start_range,
                              &junction)) {
    ADEBUG << "Dead end found, but too far, distance larger than "
              "pre-defined distance "
           << target_dead_end_id;
    return false;
  }
  return true;
}

bool DeadEndTurnAroundScenario::SelectTargetDeadEndJunction(
    std::vector<JunctionInfoConstPtr>* junctions,
    const apollo::common::PointENU& dead_end_point,
    JunctionInfoConstPtr* target_junction) {
  Vec2d start_point;
  start_point.set_x(dead_end_point.x());
  start_point.set_y(dead_end_point.y());
  // warning: the car only be the one junction
  size_t junction_num = junctions->size();
  if (junction_num <= 0) {
    ADEBUG << "No junctions frim map";
    return false;
  }
  for (size_t i = 0; i < junction_num; ++i) {
    if (junctions->at(i)->junction().type() == 5) {
      Polygon2d polygon = junctions->at(i)->polygon();
      // judge dead end point in the select junction
      if (polygon.IsPointIn(start_point)) {
        *target_junction = junctions->at(i);
        return true;
      } else {
        return false;
      }
    } else {
      ADEBUG << "No dead end junction";
      return false;
    }
  }
  return true;
}

bool DeadEndTurnAroundScenario::CheckDistanceToDeadEnd(
    const VehicleState& vehicle_state,
    const Path& nearby_path,
    const double dead_end_start_range,
    JunctionInfoConstPtr* junction) {
  const Vec2d& car_position = {vehicle_state.x(), vehicle_state.y()};
  auto junction_polygon = (*junction)->polygon();
  return std::abs(junction_polygon.DistanceTo(car_position)) <
         dead_end_start_range;
}

}  // namespace deadend_turnaround
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
