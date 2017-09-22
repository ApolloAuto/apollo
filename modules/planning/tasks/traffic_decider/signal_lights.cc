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

#include <vector>

#include "modules/planning/tasks/traffic_decider/signal_lights.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/common/adapters/adapter_manager.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::perception::TrafficLight;
using apollo::perception::TrafficLightDetection;

SignalLights::SignalLights() : TrafficRule("SignalLights") {}

bool SignalLights::ApplyRule(ReferenceLineInfo *const reference_line_info) {
  if (!FLAGS_enable_signal_lights) {
    return true;
  }
  if (!FindValidSignalLights(reference_line_info)) {
    return true;
  }
  ReadSignals();
  return true;
}

void SignalLights::ReadSignals() {
  if (!AdapterManager::GetTrafficLightDetection()->Empty()) {
    return;
  }
  const TrafficLightDetection& detection =
      AdapterManager::GetTrafficLightDetection()->GetLatestObserved();
  for (int j = 0; j < detection.traffic_light_size(); j++) {
    const TrafficLight& signal = detection.traffic_light(j);
    signals_[signal.id()] = &signal;
  }
}

bool SignalLights::FindValidSignalLights(
    ReferenceLineInfo *const reference_line_info) {
  const std::vector<hdmap::PathOverlap> &signal_lights = reference_line_info
      ->reference_line().map_path().signal_overlaps();
  if (signal_lights.size() <= 0) {
    return false;
  }
  for (const hdmap::PathOverlap &signal_light : signal_lights) {
    if (signal_light.start_s > reference_line_info->AdcSlBoundary().start_s()) {
      signal_lights_.push_back(&signal_light);
    }
  }
  return signal_lights_.size() > 0;
}

}  // namespace planning
}  // namespace apollo
