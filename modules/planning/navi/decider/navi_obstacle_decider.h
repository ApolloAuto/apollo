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

#ifndef MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_
#define MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/tasks/task.h"
/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class NaviObstacleDecider
 * @brief NaviObstacleDecider is used to make appropriate decisions for
 * obstacles around the vehicle in navigation mode.
 * Note that NaviObstacleDecider is only used in navigation mode (turn on
 * navigation mode by setting "FLAGS_use_navigation_mode" to "true") and do not
 * use it in standard mode.
 */
class NaviObstacleDecider : public Task {
 public:
  NaviObstacleDecider();

  virtual ~NaviObstacleDecider() = default;

  /**
   * @brief Get vehicle parameter
   * @return vehicle parameter
   */
  const ::apollo::common::VehicleParam &VehicleParam();

  /**
   * @brief Get unsafe obstacles' ID
   * @return unsafe_obstacle_ID_
   */
  const std::vector<std::tuple<std::string, double, double>> &UnsafeObstacles();

  /**
   * @brief get the actual nudgable distance according to the
   * position of the obstacle
   * @return actual nudgable distance
   */
  double GetNudgeDistance(
      const std::vector<const Obstacle *> &obstacles,
      const ReferenceLine &reference_line, const PathDecision &path_decision,
      const std::vector<common::PathPoint> &path_data_points,
      int *lane_obstacles_num);

  /**
   * @brief get the unsafe obstacles between trajectory and reference line.
   * @return obstacles' ID.
   */
  void GetUnsafeObstaclesInfo(
      const std::vector<common::PathPoint> &path_data_points,
      const std::vector<const Obstacle *> &obstacles);

 private:
  /**
   * @brief process path's obstacles info
   * @return Number of obstacles in the current lane
   */
  int ProcessPathObstacle(
      const std::vector<const Obstacle *> &obstacles,
      const std::vector<common::PathPoint> &path_data_points,
      const PathDecision &path_decision, const double min_lane_width);

  void JudgePointLeftOrRight(
      const common::PathPoint &projection_point,
      const std::vector<common::PathPoint> &path_data_points,
      const Obstacle *current_obstacle, const double proj_len, double *dist);
  /**
   * @brief Get the minimum path width
   * @return minimum path width
   */
  double GetMinLaneWidth(const std::vector<common::PathPoint> &path_data_points,
                         const ReferenceLine &reference_line);

 private:
  std::map<double, double> obstacle_lat_dist_;
  std::vector<std::tuple<std::string, double, double>> unsafe_obstacle_info_;

  // TODO(all): Add your member functions and variables.
};

inline const ::apollo::common::VehicleParam &
NaviObstacleDecider::VehicleParam() {
  const auto &vehicle_param = apollo::common::VehicleConfigHelper::instance()
                                  ->GetConfig()
                                  .vehicle_param();
  return vehicle_param;
}

inline const std::vector<std::tuple<std::string, double, double>>
    &NaviObstacleDecider::UnsafeObstacles() {
  return unsafe_obstacle_info_;
}

}  // namespace planning
}  // namespace apollo

#endif /* MODULES_PLANNING_NAVI_NAVI_OBSTACLE_DECIDER_H_ */
