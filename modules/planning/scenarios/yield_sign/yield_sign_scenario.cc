/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <string>

#include "modules/planning/scenarios/yield_sign/yield_sign_scenario.h"

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/scenarios/yield_sign/stage_approach.h"
#include "modules/planning/scenarios/yield_sign/stage_creep.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

bool YieldSignScenario::Init(std::shared_ptr<DependencyInjector> injector,
                             const std::string& name) {
  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioYieldSignConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }
  return true;
}

bool YieldSignScenario::IsTransferable(const Scenario* other_scenario,
                                       const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  hdmap::PathOverlap* yield_sign_overlap = nullptr;
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN) {
      return false;
    } else if (overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      yield_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    }
  }
  if (yield_sign_overlap == nullptr) {
    return false;
  }
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_yield_sign =
      yield_sign_overlap->start_s - adc_front_edge_s;
  const bool yield_sign_scenario =
      (adc_distance_to_yield_sign > 0.0 &&
       adc_distance_to_yield_sign <=
           context_.scenario_config.start_yield_sign_scenario_distance());
  return yield_sign_scenario;
}

bool YieldSignScenario::Exit(Frame* frame) {
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_yield_sign()
      ->Clear();
  return true;
}

bool YieldSignScenario::Enter(Frame* frame) {
  // get first_encountered yield_sign
  const auto& reference_line_info = frame->reference_line_info().front();
  std::string current_yield_sign_overlap_id;
  const auto& overlaps = reference_line_info.FirstEncounteredOverlaps();
  for (auto overlap : overlaps) {
    if (overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      current_yield_sign_overlap_id = overlap.second.object_id;
      break;
    }
  }

  if (current_yield_sign_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    AERROR << "Can not find yield sign overlap in reference line!";
    return false;
  }

  // find all the yield_sign at/within the same location/group
  const std::vector<hdmap::PathOverlap>& yield_sign_overlaps =
      reference_line_info.reference_line().map_path().yield_sign_overlaps();
  auto yield_sign_overlap_itr = std::find_if(
      yield_sign_overlaps.begin(), yield_sign_overlaps.end(),
      [&current_yield_sign_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_yield_sign_overlap_id;
      });
  if (yield_sign_overlap_itr == yield_sign_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    AERROR << "Can not find yield sign overlap "
           << current_yield_sign_overlap_id;
    return false;
  }

  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_yield_sign_overlap_start_s =
      yield_sign_overlap_itr->start_s;
  context_.current_yield_sign_overlap_ids.clear();
  for (const auto& yield_sign_overlap : yield_sign_overlaps) {
    const double dist =
        yield_sign_overlap.start_s - current_yield_sign_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_yield_sign()
          ->add_current_yield_sign_overlap_id(yield_sign_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered yield_sign["
             << yield_sign_overlap.object_id << "] start_s["
             << yield_sign_overlap.start_s << "]";
      context_.current_yield_sign_overlap_ids.push_back(
          yield_sign_overlap.object_id);
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
