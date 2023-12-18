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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/proto/traffic_rules_pipeline.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

/**
 * @class TrafficDecider
 * @brief Create traffic related decision in this class.
 * The created obstacles is added to obstacles_, and the decision is added to
 * obstacles_
 * Traffic obstacle examples include:
 *  * Traffic Light
 *  * End of routing
 *  * Select the drivable reference line.
 */
class TrafficDecider {
 public:
  TrafficDecider() = default;
  bool Init(const std::shared_ptr<DependencyInjector> &injector);
  virtual ~TrafficDecider() = default;
  apollo::common::Status Execute(Frame *frame,
                                 ReferenceLineInfo *reference_line_info);

 private:
  bool init_ = false;
  void BuildPlanningTarget(ReferenceLineInfo *reference_line_info);
  std::vector<std::shared_ptr<TrafficRule>> rule_list_;
  TrafficRulesPipeline rule_pipeline_;
};

}  // namespace planning
}  // namespace apollo
