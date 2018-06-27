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
  enum ValidateStopPointCode {
    OK = 0,
    OUT_OF_REFERENCE_LINE = 1,
    PASS_DEST_POINT_TOO_FAR = 2,
    BEHIND_ADC = 3,
    OPERATION_LENGTH_NOT_ENOUGH = 4,
    PARKING_SPOT_NOT_AVAIL = 5,
  };

  /**
   * Check if the planning status is in pull over mode
   */
  bool IsPullOver() const;

  /**
   * get a pull over stop point
   */
  int GetPullOverStop(common::PointENU* stop_point);

  /**
   * check if on a crosswalk/junction/clear_area/speedbumper/etc overlap
   */
  bool OnOverlap(const double start_s, const double end_s);

  /**
   * Find a safe place to pull over based on the vehicle's current state.
   */
  int FindPullOverStop(const double stop_point_s,
                       common::PointENU* stop_point);
  int FindPullOverStop(common::PointENU* stop_point);

  /**
   * Check if a stop point is valid based on current vehicle status
   * The stop point could be invalid if it is occupied by other obstacles;
   * The stop point could be invalid if the vehicle has passed this point
   */
  ValidateStopPointCode IsValidStop(const common::PointENU& stop_point) const;
  ValidateStopPointCode IsValidStop(const common::SLPoint& stop_point_sl) const;

  /**
   * handle when pull-over is completed
   */
  bool CheckPullOverComplete();

  /**
   * build stop decision
   */
  int BuildPullOverStop(const common::PointENU& stop_point);
  int BuildInLaneStop(const common::PointENU& pull_over_stop_point);
  int BuildStopDecision(const std::string& vistual_obstacle_id_postfix,
                        const double stop_line_s,
                        const common::PointENU& stop_point,
                        const double stop_point_heading);

 private:
  static constexpr char const* const PULL_OVER_VO_ID_PREFIX = "PO_";
  static constexpr char const* const INLANE_STOP_VO_ID_POSTFIX = "_INLANE";
  static constexpr double PARKING_SPOT_LONGITUDINAL_BUFFER = 1.0;
  static uint32_t failure_count_;
  static common::PointENU stop_point_;
  static common::PointENU inlane_adc_potiion_stop_point_;
  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_PULLOVER_H_
