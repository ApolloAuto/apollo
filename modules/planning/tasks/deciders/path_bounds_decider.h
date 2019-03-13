/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <functional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "modules/planning/proto/decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

class PathBoundsDecider : public Decider {
 public:
  explicit PathBoundsDecider(const TaskConfig& config);

 private:
  common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  std::string GeneratePathBoundaries(
      Frame* frame, ReferenceLineInfo* reference_line_info,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  std::string GenerateFallbackPathBoundaries(
      Frame* frame, ReferenceLineInfo* reference_line_info,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  bool InitPathBoundaries(
      const ReferenceLine& reference_line,
      const common::TrajectoryPoint& planning_start_point,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  bool GetBoundariesFromLanesAndADC(
      const ReferenceLine& reference_line,
      int lane_borrowing, double ADC_buffer,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  bool GetBoundariesFromStaticObstacles(
      PathDecision* const path_decision,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  bool GetLaneInfoFromPoint(
      double point_x, double point_y, double point_z, double point_theta,
      hdmap::LaneInfoConstPtr *const lane);

  double GetBufferBetweenADCCenterAndEdge();

  std::vector<std::tuple<int, double, double, double, std::string>>
  SortObstaclesForSweepLine(
      const IndexedList<std::string, Obstacle>& indexed_obstacles);

  std::vector<std::vector<std::tuple<double, double, double>>>
  ConstructSubsequentPathBounds(
      const std::vector<std::tuple<int, double, double, double, std::string>>&
      sorted_obstacles,
      size_t path_idx, size_t obs_idx,
      std::unordered_map<std::string, std::tuple<bool, double>>*
      const obs_id_to_details,
      std::vector<std::tuple<double, double, double>>* const curr_path_bounds);

  std::vector<std::vector<bool>> DecidePassDirections(
      double l_min, double l_max,
      const std::vector<std::tuple<int, double, double, double, std::string>>&
      new_entering_obstacles);

  /**
    * @brief Update the path_boundary at "idx", as well as the new center-line.
    *        It also checks if ADC is blocked (lmax < lmin).
    * @param The current index of the path_bounds
    * @param The minimum left boundary (l_max)
    * @param The maximum right boundary (l_min)
    * @param The path_boundaries (its content at idx will be updated)
    * @param The center_line (to be updated)
    * @return If path is good, true; if path is blocked, false.
    */
  bool UpdatePathBoundaryAndCenterLine(
      size_t idx, double left_bound, double right_bound,
      std::vector<std::tuple<double, double, double>>* const path_boundaries,
      double* const center_line);

  void TrimPathBounds(
      const int path_blocked_idx,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  void PathBoundsDebugString(
      const std::vector<std::tuple<double, double, double>>& path_boundaries);

 private:
  std::string blocking_obstacle_id_ = "";
  double adc_frenet_s_ = 0.0;
  double adc_frenet_sd_ = 0.0;
  double adc_frenet_l_ = 0.0;
  double adc_frenet_ld_ = 0.0;
  double adc_lane_width_ = 0.0;
  hdmap::LaneInfoConstPtr adc_lane_info_;
};

}  // namespace planning
}  // namespace apollo
