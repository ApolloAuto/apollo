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

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_KEEP_CLEAR_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_KEEP_CLEAR_H_

#include <string>

#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

/**
 * This class creates a virtual obstacle for each clear area region.
 */
class KeepClear : public TrafficRule {
 public:
  explicit KeepClear(const TrafficRuleConfig& config);
  virtual ~KeepClear() = default;

  bool ApplyRule(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);

 private:
  bool BuildKeepClearObstacle(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info,
                              hdmap::PathOverlap* const keep_clear_overlap,
                              const std::string& virtual_obstacle_id);
 private:
  static constexpr char const* const KEEP_CLEAR_VO_ID_PREFIX = "KC_";
  static constexpr char const* const KEEP_CLEAR_JUNCTION_VO_ID_PREFIX =
      "KC_JC_";
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_KEEP_CLEAR_H_
