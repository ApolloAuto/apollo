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

#include "modules/planning/lattice/behavior/signal_light_scenario.h"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
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

  StopPoint stop_point;
  stop_point.set_s(std::numeric_limits<double>::max());
  for (const auto signal_from_map : signals_from_map) {
    auto it_signal = detected_signals.find(signal_from_map->object_id);
    if (it_signal == detected_signals.end()) {
      AWARN << "Cannot detect signal which is marked on the map; signal id: "
            << signal_from_map->object_id;
      continue;
    }

    if (signal_from_map->start_s < stop_point.s()) {
      stop_point.set_s(signal_from_map->start_s);
      if (it_signal->second->color() == TrafficLight::RED) {
        stop_point.set_type(StopPoint::HARD);
      } else if (it_signal->second->color() == TrafficLight::YELLOW) {
        stop_point.set_type(StopPoint::SOFT);
      }
    }
  }

  if (stop_point.s() < std::numeric_limits<double>::max() &&
      stop_point.has_type()) {
    const auto& vehicle_config =
        common::VehicleConfigHelper::instance()->GetConfig();
    double front_edge_to_center =
        vehicle_config.vehicle_param().front_edge_to_center();
    planning_target->mutable_stop_point()->set_s(stop_point.s() -
                                                 front_edge_to_center);
    planning_target->mutable_stop_point()->set_type(stop_point.type());
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
    if (signal_light.start_s + FLAGS_signal_light_min_pass_s_distance >
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

}  // namespace planning
}  // namespace apollo
