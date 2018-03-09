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

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_TRAFFIC_DECIDER_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_TRAFFIC_DECIDER_H_

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/traffic_rule_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

/**
 * @class TrafficDecider
 * @brief Create traffic related decision in this class.
 * The created obstacles is added to obstacles_, and the decision is added to
 * path_obstacles_
 * Traffic obstacle examples include:
 *  * Traffic Light
 *  * End of routing
 *  * Select the drivable reference line.
 */
class TrafficDecider {
 public:
  TrafficDecider() = default;
  bool Init(const TrafficRuleConfigs &config);
  virtual ~TrafficDecider() = default;
  apollo::common::Status Execute(Frame *frame,
                                 ReferenceLineInfo *reference_line_info);

 private:
  static apollo::common::util::Factory<
      TrafficRuleConfig::RuleId, TrafficRule,
      TrafficRule *(*)(const TrafficRuleConfig &config)>
      s_rule_factory;

  void RegisterRules();
  void BuildPlanningTarget(ReferenceLineInfo *reference_line_info);

  TrafficRuleConfigs rule_configs_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_TRAFFIC_DECIDER_H_
