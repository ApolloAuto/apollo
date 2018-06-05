/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_PULLOVER_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_PULLOVER_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

/**
 * Pull Over is an action that can be triggered by other traffic rules, or
 * remote inputs, e.g., hearing a police siren.
 *
 * This class will update the current vehicle pull over state, and find
 * appropriate stop points for the vehicle to stop.
 */

class PullOver : public TrafficRule {
 public:
  explicit PullOver(const TrafficRuleConfig& config);
  virtual ~PullOver() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

 private:
  /**
   * Check if the planning status is in pull over mode
   */
  bool IsPullOver() const;

  bool PullOverCompleted();

  /**
   * get a pull over stop point
   */
  int GetPullOverStop(common::PointENU* stop_point);

  bool OnOverlap(const double s);

  /**
   * Find a safe place to pull over based on the vehicle's current state.
   */
  int FindPullOverStop(double* stop_point_s);
  int FindPullOverStop(common::SLPoint* stop_point_sl);

  /**
   * Check if a stop point is valid based on current vehicle status
   * The stop point could be invalid if it is occupied by other obstacles;
   * The stop point could be invalid if the vehicle has passed this point
   */
  bool IsValidStop() const;

  int BuildPullOverStop(const common::PointENU stop_point);

 private:
  static constexpr char const* const PULL_OVER_VO_ID_PREFIX = "PO_";
  static constexpr double PARKING_SPOT_LONGITUDINAL_BUFFER = 1.0;
  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_PULLOVER_H_
