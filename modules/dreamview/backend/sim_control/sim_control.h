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

#include "gtest/gtest_prod.h"

#include "modules/map/relative_map/proto/navigation.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/sim_control/sim_control_interface.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class SimControl
 * @brief A module that simulates a 'perfect control' algorithm, which assumes
 * an ideal world where the car can be perfectly placed wherever the planning
 * asks it to be, with the expected speed, acceleration, etc.
 */
class SimControl : SimControlInterface {
 public:
  /**
   * @brief Constructor of SimControl.
   * @param map_service the pointer of MapService.
   */
  explicit SimControl(const MapService *map_service);

  bool IsEnabled() const { return enabled_; }

  /**
   * @brief setup callbacks and timer
   * @param set_start_point initialize localization.
   */
  void Init(bool set_start_point, double start_velocity = 0.0,
            double start_acceleration = 0.0) override;

  /**
   * @brief Starts the timer to publish simulated localization and chassis
   * messages.
   */
  void Start() override;

  /**
   * @brief Stops the timer.
   */
  void Stop() override;

  /**
   * @brief Resets the internal state.
   */
  void Reset() override;

  void RunOnce() override;

 private:
  void OnPlanning(const apollo::planning::ADCTrajectory &trajectory);
  void OnRoutingResponse(const apollo::routing::RoutingResponse &routing);
  void OnReceiveNavigationInfo(
      const relative_map::NavigationInfo &navigation_info);

  /**
   * @brief Predict the next trajectory point using perfect control model
   */
  bool PerfectControlModel(apollo::common::TrajectoryPoint *point);

  void PublishChassis(double cur_speed);

  void PublishLocalization(const apollo::common::TrajectoryPoint &point);

  // Reset the start point, which can be a dummy point on the map or received
  // from the routing module.
  void SetStartPoint(const apollo::common::TrajectoryPoint &point);

  void Freeze();

  void TimerCallback(const ros::TimerEvent &event);

  void ClearPlanning();

  const MapService *map_service_ = nullptr;

  // The timer to publish simulated localization and chassis messages.
  ros::Timer sim_control_timer_;

  // Time interval of the timer, in seconds.
  static constexpr double kSimControlInterval = 0.01;

  // The latest received planning trajectory.
  apollo::planning::ADCTrajectory current_trajectory_;
  // The index of the previous and next point with regard to the
  // current_trajectory.
  int prev_point_index_ = 0;
  int next_point_index_ = 0;

  // Whether there's a planning received after the most recent routing.
  bool received_planning_ = false;

  // Number of planning received in terms of one RoutingResponse.
  int planning_count_ = -1;

  bool re_routing_triggered_ = false;

  // Whether the sim control is enabled / initialized.
  bool enabled_ = false;
  bool inited_ = false;

  // The header of the routing planning is following.
  apollo::common::Header current_routing_header_;

  apollo::common::TrajectoryPoint prev_point_;
  apollo::common::TrajectoryPoint next_point_;

  common::PathPoint adc_position_;

  // Initial velocity and acceleration of the main vehicle
  double start_velocity_ = 0.0;
  double start_acceleration_ = 0.0;

  static constexpr int kPlanningCountToStart = 5;

  relative_map::NavigationInfo navigation_info_;

  FRIEND_TEST(SimControlTest, Test);
};

}  // namespace dreamview
}  // namespace apollo

#endif  // MODULES_DREAMVIEW_BACKEND_SIM_CONTROL_SIM_CONTROL_H_
