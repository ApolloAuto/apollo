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

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_CHANGE_LANE_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_CHANGE_LANE_H_

#include <string>
#include <vector>

#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

/**
 * @brief This class defines rule-based lane change behaviors for the ADC.
 */
class ChangeLane : public TrafficRule {
 public:
  explicit ChangeLane(const TrafficRuleConfig& config);
  virtual ~ChangeLane() = default;

  bool ApplyRule(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);

 private:
  /**
   * @brief This function will filter obstacles based on change lane strategy.
   **/
  bool FilterObstacles(ReferenceLineInfo* reference_line_info);
  /**
   * @brief This function will extend the prediction of the guard obstacle to
   *guard lane change action. Due to the ST path may drive on the forward lane
   *first, then slowly move to the target lane when making lane change, we need
   *to make sure the vehicle is aware that it actually occupies the target lane,
   *even when it is not on the target lane yet.
   **/
  bool CreateGuardObstacle(const ReferenceLineInfo* reference_line_info,
                           Obstacle* obstacle);

  /**
   * @brief create overtake decision for the give path obstacle
   */
  ObjectDecisionType CreateOvertakeDecision(
      const ReferenceLine& reference_line,
      const PathObstacle* path_obstacle) const;

  std::vector<const PathObstacle*> guard_obstacles_;
  std::vector<const PathObstacle*> overtake_obstacles_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_CHANGE_LANE_H_
