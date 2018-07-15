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

#include <functional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/common/status/status.h"
#include "modules/planning/common/speed/speed_data.h"
#include "modules/planning/navi/decider/navi_obstacle_decider.h"
#include "modules/planning/navi/decider/navi_speed_ts_graph.h"
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

 private:
  /**
   * @brief Create speed-data.
   * @param start_v V of planning start point.
   * @param start_a A of planning start point.
   * @param start_da Da of planning start point.
   * @param planning_length Planning length.
   * @param path_data_points The path data of current reference line.
   * @param obstacles Current obstacles.
   * @param find_obstacle Find obstacle from id.
   * @param speed_data Output.
   * @return Status::OK() if a suitable speed-data is created; error otherwise.
   */
  apollo::common::Status MakeSpeedDecision(
      double start_v, double start_a, double start_da, double planning_length,
      const std::vector<common::PathPoint>& path_data_points,
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
   * @param path_data_points Current path data.
   * @param obstacles Current obstacles.
   * @param find_obstacle Find obstacle from id.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status AddObstaclesConstraints(
      double vehicle_speed,
      const std::vector<common::PathPoint>& path_data_points,
      const std::vector<const Obstacle*>& obstacles,
      const std::function<const Obstacle*(const std::string& id)>&
          find_obstacle);

  /**
   * @brief Add t-s constraints base on bends.
   * @return Status::OK() if success; error otherwise.
   */
  apollo::common::Status AddBendConstraints();

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
  double max_accel_;
  double max_decel_;
  double obstacle_buffer_;
  double safe_distance_base_;
  double safe_distance_ratio_;

  NaviObstacleDecider obstacle_decider_;
  NaviSpeedTsGraph ts_graph_;

  FRIEND_TEST(NaviSpeedDeciderTest, CreateSpeedData);
  FRIEND_TEST(NaviSpeedDeciderTest, CreateSpeedDataForStaticObstacle);
  FRIEND_TEST(NaviSpeedDeciderTest, CreateSpeedDataForObstacles);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_NAVI_NAVI_SPEED_DECIDER_H_
