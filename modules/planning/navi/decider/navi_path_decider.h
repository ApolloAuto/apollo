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

#ifndef MODULES_PLANNING_NAVI_NAVI_PATH_DECIDER_H_
#define MODULES_PLANNING_NAVI_NAVI_PATH_DECIDER_H_

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "gflags/gflags.h"
#include "gtest/gtest_prod.h"

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/proto/navi_path_decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/tasks/task.h"

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
class NaviPathDecider : public Task {
 public:
  NaviPathDecider();
  virtual ~NaviPathDecider() = default;

  bool Init(const PlanningConfig &config) override;

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
   * @brief shift the path points on the y-axis
   * @param shift_distance shift distance in y-axis.
   * @param init_point both input and output path points.
   * @return if success return true or return false.
   */
  void ShiftY(const double shift_distance,
              std::vector<common::PathPoint> *const path_points);

  /**
   * @brief calculate the y-coordinate of the starting point of the path plan
   * @param actual_ref_init_y the actual y-coordinate of start point that
   * intercepted from reference line
   * @param target_path_init_y the y-coordinate of the start point that desired
   * @return the y-coordinate of the starting point in FLU coordinate.
   */
  double SmoothInitY(const double actual_ref_init_y,
                     const double target_path_init_y);

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
   * @return the y coordinate value of nudging target position
   */
  double NudgeProcess(const ReferenceLine &reference_line,
                      const std::vector<common::PathPoint> &path_data_points,
                      const std::vector<const Obstacle *> &obstacles,
                      const PathDecision &path_decision);
  /**
   * @brief calculate latreal shift param by vehicle state and config
   */
  void CalculateShiftParam();

 private:
  common::VehicleState vehicle_state_;
  NaviPathDeciderConfig config_;
  std::string cur_reference_line_lane_id_;
  std::map<std::string, double> last_lane_id_to_adc_project_y_;
  std::map<std::string, bool> last_lane_id_to_nudge_flag_;
  std::map<double, std::tuple<double, double, double>>
      speed_to_shift_param_table_;
  std::vector<double> max_speed_levels_;
  double theta_change_ratio_ = 0.0;
  double min_init_y_ = 0.0;
  double max_init_y_ = 0.0;

  FRIEND_TEST(NaviPathDeciderTest, SmoothInitY);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_NAVI_NAVI_PATH_DECIDER_H_
