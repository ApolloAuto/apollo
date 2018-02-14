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

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_SIDEPASS_VEHICE_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_SIDEPASS_VEHICE_H_

#include <string>

#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

class SidepassVehicle : public TrafficRule {
 public:
  explicit SidepassVehicle(const RuleConfig& config);
  virtual ~SidepassVehicle() = default;

  bool ApplyRule(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);

  // a sidepass sequence includes:
  // driving -> wait -> sidepass -> driving
  enum class SidepassStatus {
    UNKNOWN = 0,
    DRIVING = 1,
    WAIT = 2,
    SIDEPASS = 3,
  };

 private:
  bool UpdateSidepassStatus(const SLBoundary& adc_sl_boundary,
                            const common::TrajectoryPoint& adc_planning_point,
                            PathDecision* path_decision);

  bool HasBlockingObstacle(const SLBoundary& adc_sl_boundary,
                           const PathDecision& path_decision);

  /**
   * @brief When the reference line info indicates that there is no lane change,
   * use lane keeping strategy for back side vehicles.
   */
  bool MakeSidepassObstacleDecision(
      const SLBoundary& adc_sl_boundary,
      const common::TrajectoryPoint& adc_planning_point,
      PathDecision* path_decision);

  constexpr static char const* const db_key_sidepass_status =
      "DROPBOX_KEY_SIDEPASS_STATUS";
  constexpr static char const* const db_key_sidepass_adc_wait_start_time =
      "DROPBOX_KEY_SIDEPASS_OBSTACLE_ADC_WAIT_TIME";
  constexpr static char const* const db_key_sidepass_obstacle_id =
      "DROPBOX_KEY_SIDEPASS_OBSTACLE_ID";
  constexpr static char const* const db_key_sidepass_side =
      "DROPBOX_KEY_SIDEPASS_SIDE";

  const hdmap::HDMap* hdmap_ = nullptr;
  const ReferenceLine* reference_line_ = nullptr;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_SIDEPASS_VEHICE_H_
