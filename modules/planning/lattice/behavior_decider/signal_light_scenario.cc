/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/lattice/behavior_decider/signal_light_scenario.h"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/map_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_internal.pb.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::common::util::WithinBound;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

int SignalLightScenario::ComputeScenarioDecision(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    PlanningTarget* planning_target) {
  CHECK(frame != nullptr);

  auto signals_from_map = FindValidSignalLightFromMap(reference_line_info);
  if (signals_from_map.empty()) {
    ADEBUG << "No valid signal light along reference line";
    return 0;
  }
  auto detected_signals = GetPerceptionDetectedSignals();

  double stop_s = std::numeric_limits<double>::max();
  for (const auto signal_from_map : signals_from_map) {
    auto it_signal = detected_signals.find(signal_from_map->object_id);
    if (it_signal == detected_signals.end()) {
      AWARN << "Cannot detect signal which is marked on the map; signal id: "
          << signal_from_map->object_id;
      continue;
    }

    double stop_deceleration =
        EstimateStopDeceleration(reference_line_info, signal_from_map);

    if ((it_signal->second->color() == TrafficLight::RED &&
         stop_deceleration < FLAGS_max_stop_deceleration) ||
        // (signal.color() == TrafficLight::UNKNOWN &&
        //  stop_deceleration < FLAGS_max_stop_deceleration) ||
        (it_signal->second->color() == TrafficLight::YELLOW &&
         stop_deceleration < FLAGS_max_stop_deacceleration_for_yellow_light)) {
      CreateStopObstacle(frame, reference_line_info, signal_from_map);

      stop_s = std::min(stop_s, signal_from_map->start_s);
    }
  }

  if (stop_s < std::numeric_limits<double>::max()) {
    planning_target->set_stop_point(stop_s);
  }

  return 0;
}

std::vector<const hdmap::PathOverlap*>
SignalLightScenario::FindValidSignalLightFromMap(
    ReferenceLineInfo* const reference_line_info) {

  const std::vector<hdmap::PathOverlap>& signal_lights =
      reference_line_info->reference_line().map_path().signal_overlaps();
  if (signal_lights.size() == 0) {
    ADEBUG << "No signal lights from reference line.";
    return std::vector<const hdmap::PathOverlap*>();
  }
  ADEBUG << "Found signal_lights size=" << signal_lights.size();

  std::vector<const hdmap::PathOverlap*> signal_lights_along_reference_line;
  for (const auto& signal_light : signal_lights) {
    if (signal_light.start_s + FLAGS_max_stop_distance_buffer >
        reference_line_info->AdcSlBoundary().end_s()) {
      signal_lights_along_reference_line.push_back(&signal_light);
    }
  }
  return signal_lights_along_reference_line;
}

std::unordered_map<std::string, const TrafficLight*>
SignalLightScenario::GetPerceptionDetectedSignals() {
  if (AdapterManager::GetTrafficLightDetection()->Empty() ||
      (AdapterManager::GetTrafficLightDetection()->GetDelaySec() >
            FLAGS_signal_expire_time_sec)) {
    ADEBUG << "traffic light signals msg is either empty or outdated.";
    return std::unordered_map<std::string, const TrafficLight*>();
  }

  std::unordered_map<std::string, const TrafficLight*> detected_signals;
  const auto& signal_msg =
      AdapterManager::GetTrafficLightDetection()->GetLatestObserved();

  for (int j = 0; j < signal_msg.traffic_light_size(); ++j) {
    const auto& signal = signal_msg.traffic_light(j);
    detected_signals[signal.id()] = &signal;
  }
  return detected_signals;
}

double SignalLightScenario::EstimateStopDeceleration(
    ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* signal_light) {
  double ego_speed =
      common::VehicleStateProvider::instance()->linear_velocity();

  double ego_front_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_s = signal_light->start_s;

  if (stop_s >= ego_front_s) {
    double stop_distance = stop_s - ego_front_s;
    return (ego_speed * ego_speed) / stop_distance * 0.5;
  } else {
    // longitudinal_acceleration_lower_bound is a negative value.
    return -FLAGS_longitudinal_acceleration_lower_bound;
  }
}

void SignalLightScenario::CreateStopObstacle(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    const hdmap::PathOverlap* signal_light) {
  // check
  const auto& reference_line = reference_line_info->reference_line();
  const double stop_s =
      signal_light->start_s - FLAGS_traffic_light_stop_distance;
  const double box_center_s =
      signal_light->start_s + FLAGS_virtual_stop_wall_length / 2.0;
  if (!WithinBound(0.0, reference_line.Length(), stop_s) ||
      !WithinBound(0.0, reference_line.Length(), box_center_s)) {
    ADEBUG << "signal " << signal_light->object_id
           << " is not on reference line";
    return;
  }

  // create virtual stop wall
  std::string virtual_obstacle_id =
      FLAGS_signal_light_virtual_obstacle_id_prefix + signal_light->object_id;
  auto* obstacle = frame->CreateVirtualStopObstacle(
      reference_line_info, virtual_obstacle_id, signal_light->start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
    return;
  }

  return;
}

}  // namespace planning
}  // namespace apollo
