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
 * @brief This file provides the declaration of the class "NaviPathDecider".
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include "gtest/gtest_prod.h"

#include "gflags/gflags.h"

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/planning/planning_base/proto/planning_config.pb.h"

#include "modules/common/status/status.h"
#include "modules/planning/planners/navi/decider/navi_obstacle_decider.h"
#include "modules/planning/planners/navi/decider/navi_task.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class NaviPathDecider
 * @brief NaviPathDecider is used to generate the local driving path of the
.* vehicle in navigation mode.
 * Note that NaviPathDecider is only used in navigation mode (turn on navigation
 * mode by setting "FLAGS_use_navigation_mode" to "true") and do not use it in
 * standard mode.
 */
class NaviPathDecider : public NaviTask {
 public:
  NaviPathDecider();
  virtual ~NaviPathDecider() = default;

  bool Init(const PlannerNaviConfig &config) override;

  /**
   * @brief Overrided implementation of the virtual function "Execute" in the
   * base class "Task".
   * @param frame Current planning frame.
   * @param reference_line_info Currently available reference line information.
   * @return Status::OK() if a suitable path is created; error otherwise.
   */
  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  /**
   * @brief generate path information for trajectory plan in navigation mode.
   * @param reference_line  the reference line.
   * @param init_point start planning point.
   * @param obstacles unhandled obstacle information.
   * @param path_decision path decision information provided by perception.
   * @param path_data output path plan information based on FLU coordinate
   * system
   * @return Status::OK() if a suitable path is created; error otherwise.
   */
  apollo::common::Status Process(const ReferenceLine &reference_line,
                                 const common::TrajectoryPoint &init_point,
                                 const std::vector<const Obstacle *> &obstacles,
                                 PathDecision *const path_decision,
                                 PathData *const path_data);

  /**
   * @brief take a section of the reference line as the initial path trajectory.
   * @param reference_line input reference line.
   * @param path_points output points intercepted from the reference line
   * @return if success return true or return false.
   */
  bool GetBasicPathData(const ReferenceLine &reference_line,
                        std::vector<common::PathPoint> *const path_points);

  /**
   * @brief if adc is not on the dest lane, move to dest lane slowly.
   * @param the y of adc project point to dest lane reference line.
   * @param path point intercepted from the reference line
   */
  void MoveToDestLane(const double dest_ref_line_y,
                      std::vector<common::PathPoint> *const path_points);

  /**
   * @brief if adc is on the dest lane, keep lane.
   * @param the y of adc project point to dest lane reference line.
   * @param path point intercepted from the reference line
   */
  void KeepLane(const double dest_ref_line_y,
                std::vector<common::PathPoint> *const path_points);

  void RecordDebugInfo(const PathData &path_data);

  /**
   * @brief check whether it is safe to change lanes
   * @param reference_line input change lane reference line
   * @param path_decision input all abstacles info
   * @return true if safe to change lane or return false.
   */
  bool IsSafeChangeLane(const ReferenceLine &reference_line,
                        const PathDecision &path_decision);

  /**
   * @brief calculate the lateral target position with slight avoidance
   * @path_data_points the basic path data intercepted from the reference line
   * @param reference_line input reference line
   * @param obstacles unhandled obstacle information.
   * @param path_decision path decision information provided by perception.
   * @vehicle_state adc status
   * @return the y coordinate value of nudging target position
   */
  double NudgeProcess(const ReferenceLine &reference_line,
                      const std::vector<common::PathPoint> &path_data_points,
                      const std::vector<const Obstacle *> &obstacles,
                      const PathDecision &path_decision,
                      const common::VehicleState &vehicle_state);
  /**
   * @brief calculate latreal shift distance by vehicle state and config
   */
  double CalculateDistanceToDestLane();

 private:
  double max_keep_lane_distance_ = 0.0;
  double min_keep_lane_offset_ = 0.0;
  double max_keep_lane_shift_y_ = 0.0;
  double keep_lane_shift_compensation_ = 0.0;
  double move_dest_lane_compensation_ = 0.0;
  uint32_t start_plan_point_from_ = 0;
  std::map<double, double> move_dest_lane_config_talbe_;
  std::vector<double> max_speed_levels_;

  double start_plan_v_ = 0.0;
  double start_plan_a_ = 0.0;
  apollo::common::PathPoint start_plan_point_;

  std::string cur_reference_line_lane_id_;
  std::map<std::string, bool> last_lane_id_to_nudge_flag_;
  NaviObstacleDecider obstacle_decider_;
  common::VehicleState vehicle_state_;
  NaviPathDeciderConfig config_;

  FRIEND_TEST(NaviPathDeciderTest, MoveToDestLane);
  FRIEND_TEST(NaviPathDeciderTest, KeepLane);
};

}  // namespace planning
}  // namespace apollo
