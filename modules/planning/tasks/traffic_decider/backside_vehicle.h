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

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_BACKSIDE_VEHICE_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_BACKSIDE_VEHICE_H_

#include <string>

#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

class BacksideVehicle : public TrafficRule {
 public:
  explicit BacksideVehicle(const TrafficRuleConfig& config);
  virtual ~BacksideVehicle() = default;

  bool ApplyRule(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);

 private:
  /**
   * @brief When the reference line info indicates that there is no lane change,
   * use lane keeping strategy for back side vehicles.
   */
  void MakeLaneKeepingObstacleDecision(const SLBoundary& adc_sl_boundary,
                                       PathDecision* path_decision);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_BACK_SIDEVEHICE_H_
