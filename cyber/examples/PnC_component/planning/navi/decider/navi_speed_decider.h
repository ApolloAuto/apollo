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

#pragma once

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/navi/decider/navi_obstacle_decider.h"
#include "modules/planning/navi/decider/navi_speed_ts_graph.h"
#include "modules/planning/navi/decider/navi_task.h"
#include "modules/planning/proto/planning_config.pb.h"

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
class NaviSpeedDecider : public NaviTask {
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

 private:
  /**
   * @brief Create speed-data.
   * @param start_v V of planning start point.
   * @param start_a A of planning start point.
   * @param start_da Da of planning start point.
   * @param path_points Current path data.
   * @param obstacles Current obstacles.
   * @param find_obstacle Find obstacle from id.
   * @param speed_data Output.
   * @return Status::OK() if a suitable speed-data is created; error otherwise.
   */
  apollo::common::Status MakeSpeedDecision(
      double start_v, double start_a, double start_da,
      const std::vector<common::PathPoint>& path_points,
      const std::vector<const Obstacle*>& obstacles,
      const std::function<const Obstacle*(const std::string& id)>&
          find_obstacle,
      SpeedData* const speed_data);

  /**
   * @brief Add t-s constraints base on range of perception.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status AddPerceptionRangeConstraints();

  /**
   * @brief Add t-s constraints base on obstacles.
   * @param vehicle_speed Current speed of vehicle.
   * @param path_length The length of path, just as an obstacle.
   * @param path_points Current path data.
   * @param obstacles Current obstacles.
   * @param find_obstacle Find obstacle from id.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status AddObstaclesConstraints(
      double vehicle_speed, double path_length,
      const std::vector<common::PathPoint>& path_points,
      const std::vector<const Obstacle*>& obstacles,
      const std::function<const Obstacle*(const std::string& id)>&
          find_obstacle);

  /**
   * @brief Add t-s constraints base on traffic decision.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status AddTrafficDecisionConstraints();

  /**
   * @brief Add t-s constraints base on centric acceleration.
   * @param path_points Current path data.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status AddCentricAccelerationConstraints(
      const std::vector<common::PathPoint>& path_points);

  /**
   * @brief Add t-s constraints base on configs, which has max-speed etc.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status AddConfiguredConstraints();

  void RecordDebugInfo(const SpeedData& speed_data);

 private:
  double preferred_speed_;
  double max_speed_;
  double preferred_accel_;
  double preferred_decel_;
  double preferred_jerk_;
  double max_accel_;
  double max_decel_;
  double obstacle_buffer_;
  double safe_distance_base_;
  double safe_distance_ratio_;
  double following_accel_ratio_;
  double soft_centric_accel_limit_;
  double hard_centric_accel_limit_;
  double hard_speed_limit_;
  double hard_accel_limit_;
  bool enable_safe_path_;
  bool enable_planning_start_point_;
  bool enable_accel_auto_compensation_;
  double kappa_preview_;
  double kappa_threshold_;

  NaviObstacleDecider obstacle_decider_;
  NaviSpeedTsGraph ts_graph_;

  double prev_v_ = 0.0;
  double accel_compensation_ratio_ = 1.0;
  double decel_compensation_ratio_ = 1.0;

  FRIEND_TEST(NaviSpeedDeciderTest, CreateSpeedData);
  FRIEND_TEST(NaviSpeedDeciderTest, CreateSpeedDataForStaticObstacle);
  FRIEND_TEST(NaviSpeedDeciderTest, CreateSpeedDataForObstacles);
  FRIEND_TEST(NaviSpeedDeciderTest, CreateSpeedDataForCurve);
};

}  // namespace planning
}  // namespace apollo
