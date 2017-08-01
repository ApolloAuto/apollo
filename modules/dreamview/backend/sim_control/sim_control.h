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
 */

#ifndef MODULES_DREAMVIEW_BACKEND_SIM_CONTROL_SIM_CONTROL_H_
#define MODULES_DREAMVIEW_BACKEND_SIM_CONTROL_SIM_CONTROL_H_

#include <string>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class SimControl
 * @brief
 */
class SimControl {
 public:
  SimControl();

  /**
   * @brief Starts the timer to publish simulated localization and chassis
   * messages.
   */
  void Start();

  /**
   * @brief Stops the timer.
   */
  void Stop();

 private:
  void OnPlanning(const apollo::planning::ADCTrajectory &trajectory);

  void SetStartPoint(const apollo::hdmap::RoutingResult &routing,
                     apollo::common::TrajectoryPoint *point);

  void Freeze();

  double AbsoluteTimeOfNextPoint();
  bool NextPointWithinRange();

  void TimerCallback(const ros::TimerEvent &event);

  void PublishChassis(double lambda);
  void PublishLocalization(double lambda);

  template <typename T>
  T Interpolate(T prev, T next, double lambda) {
    return (1 - lambda) * prev + lambda * next;
  }

  // The timer to publish simulated localization and chassis messages.
  ros::Timer sim_control_timer_;

  // Time interval of the timer, in seconds.
  static constexpr double kSimControlInterval = 0.01;

  // The latest received planning trajectory.
  apollo::planning::ADCTrajectory current_trajectory_;
  // The index of the previous and next point with regard to the
  // current_trajectory.
  int prev_point_index_;
  int next_point_index_;

  apollo::common::TrajectoryPoint prev_point_;
  apollo::common::TrajectoryPoint next_point_;
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_SIM_CONTROL_SIM_CONTROL_H_
