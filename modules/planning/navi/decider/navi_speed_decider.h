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
 * @brief This file provides the declaration of the class "NaviSpeedDecider".
 */

#ifndef MODULES_PLANNING_NAVI_NAVI_SPEED_DECIDER_H_
#define MODULES_PLANNING_NAVI_NAVI_SPEED_DECIDER_H_

#include <vector>

#include "modules/common/status/status.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class NaviSpeedDecider
 * @brief NaviSpeedDecider is used to generate an appropriate speed curve
 * of the vehicle in navigation mode.
 * Note that NaviSpeedDecider is only used in navigation mode (turn on
 * navigation mode by setting "FLAGS_use_navigation_mode" to "true") and do not
 * use it in standard mode.
 */
class NaviSpeedDecider : public Task {
 public:
  NaviSpeedDecider();
  virtual ~NaviSpeedDecider() = default;

  bool Init(const PlanningConfig& config) override;

  /**
   * @brief Overrided implementation of the virtual function "Execute" in the
   * base class "Task".
   * @param frame Current planning frame.
   * @param reference_line_info Currently available reference line information.
   * @return Status::OK() if a suitable path is created; error otherwise.
   */
  apollo::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  /**
   * @brief Update acceleration settings.
   * @param preferred_accel Preferred acceleration.
   * @param preferred_decel Preferred deacceleration.
   * @param max_accel Max acceleration.
   * @param max_decel Max deacceleration.
   */
  void UpdateAccelSettings(
      double preferred_accel,
      double preferred_decel,
      double max_accel,
      double max_decel);

  /**
   * @brief Create speed-data, used for unit test.
   * @param vehicle_state Current vehicle state.
   * @param obstacles Current obstacles.
   * @param speed_data Data to output.
   * @return Status::OK() if a suitable speed-data is created; error otherwise.
   */
  apollo::common::Status MakeSpeedDecision(
      const common::VehicleState& vehicle_state,
      const std::vector<const Obstacle*>& obstacles,
      SpeedData* const speed_data);

 private:
  void RecordDebugInfo(const SpeedData& speed_data);

 private:
  PlanningConfig config_;
  double preferred_accel_;
  double preferred_decel_;
  double max_accel_;
  double max_decel_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_NAVI_NAVI_SPEED_DECIDER_H_
