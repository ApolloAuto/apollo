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

namespace apollo {
namespace planning {

SignalLights::SignalLights() : TrafficRule("SignalLights") {}

bool SignalLights::ApplyRule(ReferenceLineInfo* const reference_line_info) {
  if (!FLAGS_enable_signal_lights) {
    return true;
  }
  const std::vector<hdmap::PathOverlap>& signals = reference_line_info
      ->reference_line().map_path().signal_overlaps();
  if (signals.size() <= 0) {
    return true;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
