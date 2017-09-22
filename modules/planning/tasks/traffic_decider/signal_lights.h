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

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_SIGNAL_LIGHTS_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_SIGNAL_LIGHTS_H_

#include <string>
#include <vector>
#include <unordered_map>

#include "modules/planning/tasks/traffic_decider/traffic_rule.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

namespace apollo {
namespace planning {

class SignalLights : public TrafficRule {
 public:
  SignalLights();
  virtual ~SignalLights() = default;

  bool ApplyRule(ReferenceLineInfo* const reference_line_info);

 private:
  void ReadSignals();
  bool FindValidSignalLights(ReferenceLineInfo* const reference_line_info);

  std::vector<const hdmap::PathOverlap*> signal_lights_;
  std::unordered_map<std::string, const apollo::perception::TrafficLight*>
      signals_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_SIGNAL_LIGHTS_H_
