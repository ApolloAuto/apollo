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

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planners/navi/proto/navi_task_config.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/planners/navi/decider/navi_task.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/obstacle.h"

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
class NaviObstacleDecider : public NaviTask {
 public:
  NaviObstacleDecider();

  virtual ~NaviObstacleDecider() = default;
  /**
   * @brief Initialization parameters
   * @return Initialization success or not
   */
  bool Init(const PlannerNaviConfig &config) override;

  /**
   * @brief get the actual nudgable distance according to the
   * position of the obstacle
   * @return actual nudgable distance
   */
  double GetNudgeDistance(
      const std::vector<const Obstacle *> &obstacles,
      const ReferenceLine &reference_line, const PathDecision &path_decision,
      const std::vector<common::PathPoint> &path_data_points,
      const common::VehicleState &vehicle_state, int *lane_obstacles_num);

  /**
   * @brief get the unsafe obstacles between trajectory and reference line.
   *
   */
  void GetUnsafeObstaclesInfo(
      const std::vector<common::PathPoint> &path_data_points,
      const std::vector<const Obstacle *> &obstacles);

  /**
   * @brief Get unsafe obstacles' ID
   * @return unsafe_obstacle_ID_
   */
  const std::vector<std::tuple<std::string, double, double>> &UnsafeObstacles();

 private:
  /**
   * @brief Get vehicle parameter
   * @return vehicle parameter
   */
  const apollo::common::VehicleParam &VehicleParam();

  /**
   * @brief Set last nudge dist
   * @
   */
  void SetLastNudgeDistance(double dist);

  /**
   * @brief process path's obstacles info
   * @return Number of obstacles in the current lane
   */
  void ProcessObstacle(const std::vector<const Obstacle *> &obstacles,
                       const std::vector<common::PathPoint> &path_data_points,
                       const PathDecision &path_decision,
                       const double min_lane_width,
                       const common::VehicleState &vehicle_state);

  /**
   * @brief According to the relation between the obstacle and the path data,
   * the sign is added to the distance away from the path data.Take positive on
   * the left and negative on the right
   */
  void AddObstacleOffsetDirection(
      const common::PathPoint &projection_point,
      const std::vector<common::PathPoint> &path_data_points,
      const Obstacle *current_obstacle, const double proj_len, double *dist);

  /**
   * @brief Remove safe obstacles
   * @return whether filter the obstacle
   */
  bool IsNeedFilterObstacle(
      const Obstacle *current_obstacle,
      const common::PathPoint &vehicle_projection_point,
      const std::vector<common::PathPoint> &path_data_points,
      const common::VehicleState &vehicle_state,
      common::PathPoint *projection_point_ptr);

  /**
   * @brief Get the minimum path width
   * @return minimum path width
   */
  double GetMinLaneWidth(const std::vector<common::PathPoint> &path_data_points,
                         const ReferenceLine &reference_line);

  /**
   * @brief Record last nudge
   *
   */
  void RecordLastNudgeDistance(const double nudge_dist);

  /**
   * @brief Get the actual distance between the vehicle and the obstacle based
   * on path data
   * @return Actual distance
   */
  double GetObstacleActualOffsetDistance(
      std::map<double, double>::iterator iter, const double right_nedge_lane,
      const double left_nudge_lane, int *lane_obstacles_num);
  /**
   * @brief Eliminate the influence of clutter signals on Nudge
   */
  void SmoothNudgeDistance(const common::VehicleState &vehicle_state,
                           double *nudge_dist);
  void KeepNudgePosition(const double nudge_dist, int *lane_obstacles_num);

 private:
  NaviObstacleDeciderConfig config_;
  std::map<double, double> obstacle_lat_dist_;
  std::vector<std::tuple<std::string, double, double>> unsafe_obstacle_info_;
  double last_nudge_dist_ = 0.0;
  unsigned int no_nudge_num_ = 0;
  unsigned int limit_speed_num_ = 0;
  unsigned int eliminate_clutter_num_ = 0;
  unsigned int last_lane_obstacles_num_ = 0;
  unsigned int statist_count_ = 0;
  unsigned int cycles_count_ = 0;
  bool is_obstacle_stable_ = false;
  bool keep_nudge_flag_ = false;

  FRIEND_TEST(NaviObstacleDeciderTest, ComputeNudgeDist1);
  FRIEND_TEST(NaviObstacleDeciderTest, ComputeNudgeDist2);
  FRIEND_TEST(NaviObstacleDeciderTest, ComputeNudgeDist3);
  FRIEND_TEST(NaviObstacleDeciderTest, ComputeNudgeDist4);
  FRIEND_TEST(NaviObstacleDeciderTest, GetUnsafeObstaclesID);
  // TODO(all): Add your member functions and variables.
};

inline const apollo::common::VehicleParam &NaviObstacleDecider::VehicleParam() {
  const auto &vehicle_param = apollo::common::VehicleConfigHelper::Instance()
                                  ->GetConfig()
                                  .vehicle_param();
  return vehicle_param;
}

inline const std::vector<std::tuple<std::string, double, double>> &
NaviObstacleDecider::UnsafeObstacles() {
  return unsafe_obstacle_info_;
}

inline void NaviObstacleDecider::SetLastNudgeDistance(double dist) {
  last_nudge_dist_ = dist;
}
}  // namespace planning
}  // namespace apollo
