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

#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

/**
 * @brief This class defines rule-based lane change behaviors for the ADC.
 */
class ChangeLane : public TrafficRule {
 public:
  explicit ChangeLane(const RuleConfig& config);
  virtual ~ChangeLane() = default;

  bool ApplyRule(Frame* frame, ReferenceLineInfo* const reference_line_info);

 private:
  /**
   * @brief This function will find vehicles that may be in change lane blind
   *zone.
   **/
  void CreateGuardObstacles();

  /**
   * @brief This function will extend the prediction of the guard obstacle to
   *guard lane change action. Due to the ST path may drive on the forward lane
   *first, then slowly move to the target lane when making lane change, we need
   *to make sure the vehicle is aware that it actually occupies the target lane,
   *even when it is not on the target lane yet.
   **/
  void CreateGuardObstacle(Obstacle* obstacle);

  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_CHANGE_LANE_H_
