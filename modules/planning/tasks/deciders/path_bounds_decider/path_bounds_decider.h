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
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "modules/planning/proto/decider_config.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"

namespace apollo {
namespace planning {

constexpr double kPathBoundsDeciderHorizon = 100.0;
constexpr double kPathBoundsDeciderResolution = 0.5;
constexpr double kDefaultLaneWidth = 5.0;
constexpr double kDefaultRoadWidth = 20.0;
// TODO(all): Update extra tail point base on vehicle speed.
constexpr int kNumExtraTailBoundPoint = 20;
constexpr double kPulloverLonSearchCoeff = 1.5;
constexpr double kPulloverLatSearchCoeff = 1.25;

class PathBoundsDecider : public Decider {
 public:
  enum class LaneBorrowInfo {
    LEFT_BORROW,
    NO_BORROW,
    RIGHT_BORROW,
  };
  explicit PathBoundsDecider(const TaskConfig& config);

 private:
  /** @brief Every time when Process function is called, it will:
   *   1. Initialize.
   *   2. Generate Fallback Path Bound.
   *   3. Generate Regular Path Bound(s).
   */
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  /////////////////////////////////////////////////////////////////////////////
  // Below are functions called every frame when executing PathBoundsDecider.

  /** @brief The initialization function.
   */
  void InitPathBoundsDecider(const Frame& frame,
                             const ReferenceLineInfo& reference_line_info);

  /** @brief The regular path boundary generation considers the ADC itself
   *   and other static environments:
   *   - ADC's position (lane-changing considerations)
   *   - lane info
   *   - static obstacles
   *   The philosophy is: static environment must be and can only be taken
   *   care of by the path planning.
   * @param reference_line_info
   * @param lane_borrow_info: which lane to borrow.
   * @param The generated regular path_boundary, if there is one.
   * @param The blocking obstacle's id. If none, then it's not modified.
   * @return A failure message. If succeeded, return "" (empty string).
   */
  std::string GenerateRegularPathBound(
      const ReferenceLineInfo& reference_line_info,
      const LaneBorrowInfo lane_borrow_info,
      std::vector<std::tuple<double, double, double>>* const path_bound,
      std::string* const blocking_obstacle_id,
      std::string* const borrow_lane_type);

  /** @brief The fallback path only considers:
   *   - ADC's position (so that boundary must contain ADC's position)
   *   - lane info
   *   It is supposed to be the last resort in case regular path generation
   *   fails so that speed decider can at least have some path and won't
   *   fail drastically.
   *   Therefore, it be reliable so that optimizer will not likely to
   *   fail with this boundary, and therefore doesn't consider any static
   *   obstacle. When the fallback path is used, stopping before static
   *   obstacles should be taken care of by the speed decider. Also, it
   *   doesn't consider any lane-borrowing.
   * @param reference_line_info
   * @param The generated fallback path_boundary, if there is one.
   * @return A failure message. If succeeded, return "" (empty string).
   */
  std::string GenerateFallbackPathBound(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* const path_bound);

  std::string GeneratePullOverPathBound(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* const path_bound);

  int IsPointWithinPathBound(
      const ReferenceLineInfo& reference_line_info, const double x,
      const double y,
      const std::vector<std::tuple<double, double, double>>& path_bound);

  bool SearchPullOverPosition(
      const Frame& frame, const ReferenceLineInfo& reference_line_info,
      const std::vector<std::tuple<double, double, double>>& path_bound,
      std::tuple<double, double, double, int>* const pull_over_configuration);

  /** @brief Remove redundant path bounds in the following manner:
   *   - if "left" is contained by "right", remove "left"; vice versa.
   */
  void RemoveRedundantPathBoundaries(
      std::vector<PathBoundary>* const candidate_path_boundaries);

  bool IsContained(const std::vector<std::pair<double, double>>& lhs,
                   const std::vector<std::pair<double, double>>& rhs);

  /////////////////////////////////////////////////////////////////////////////
  // Below are functions called when generating path bounds.
  //  1. InitPathBoundary
  //  2. GetBoundaryFromLanesAndADC
  //  3. GetBoundaryFromStaticObstacles

  /** @brief Initializes an empty path boundary.
   */
  bool InitPathBoundary(
      const ReferenceLine& reference_line,
      std::vector<std::tuple<double, double, double>>* const path_bound);

  /** @brief Refine the boundary based on the road-info.
   *  The returned boundary is with respect to the lane-center (NOT the
   *  reference_line), though for most of the times reference_line's
   *  deviation from lane-center is negligible.
   */
  bool GetBoundaryFromRoads(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* const path_bound);

  /** @brief Refine the boundary based on lane-info and ADC's location.
   *   It will comply to the lane-boundary. However, if the ADC itself
   *   is out of the given lane(s), it will adjust the boundary
   *   accordingly to include ADC's current position.
   */
  bool GetBoundaryFromLanesAndADC(
      const ReferenceLineInfo& reference_line_info,
      const LaneBorrowInfo lane_borrow_info, double ADC_buffer,
      std::vector<std::tuple<double, double, double>>* const path_bound,
      std::string* const borrow_lane_type);

  void ConvertBoundaryAxesFromLaneCenterToRefLine(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* const path_bound);

  /** @brief Refine the boundary based on static obstacles. It will make sure
   *   the boundary doesn't contain any static obstacle so that the path
   *   generated by optimizer won't collide with any static obstacle.
   */
  bool GetBoundaryFromStaticObstacles(
      const PathDecision& path_decision,
      std::vector<std::tuple<double, double, double>>* const path_boundaries,
      std::string* const blocking_obstacle_id);

  std::vector<std::tuple<int, double, double, double, std::string>>
  SortObstaclesForSweepLine(
      const IndexedList<std::string, Obstacle>& indexed_obstacles);

  std::vector<std::vector<std::tuple<double, double, double>>>
  ConstructSubsequentPathBounds(
      const std::vector<std::tuple<int, double, double, double, std::string>>&
          sorted_obstacles,
      size_t path_idx, size_t obs_idx,
      std::unordered_map<std::string, std::tuple<bool, double>>* const
          obs_id_to_details,
      std::vector<std::tuple<double, double, double>>* const curr_path_bounds);

  std::vector<std::vector<bool>> DecidePassDirections(
      double l_min, double l_max,
      const std::vector<std::tuple<int, double, double, double, std::string>>&
          new_entering_obstacles);

  /////////////////////////////////////////////////////////////////////////////
  // Below are several helper functions:

  /** @brief Get the distance between ADC's center and its edge.
   * @return The distance.
   */
  double GetBufferBetweenADCCenterAndEdge();

  /** @brief Update the path_boundary at "idx", as well as the new center-line.
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

  /** @brief Trim the path bounds starting at the idx where path is blocked.
   */
  void TrimPathBounds(
      const int path_blocked_idx,
      std::vector<std::tuple<double, double, double>>* const path_boundaries);

  /** @brief Print out the path bounds for debugging purpose.
   */
  void PathBoundsDebugString(
      const std::vector<std::tuple<double, double, double>>& path_boundaries);

 private:
  double adc_frenet_s_ = 0.0;
  double adc_frenet_sd_ = 0.0;
  double adc_frenet_l_ = 0.0;
  double adc_frenet_ld_ = 0.0;
  double adc_l_to_lane_center_ = 0.0;
  double adc_lane_width_ = 0.0;

  FRIEND_TEST(PathBoundsDeciderTest, InitPathBoundary);
  FRIEND_TEST(PathBoundsDeciderTest, GetBoundaryFromLanesAndADC);
};

}  // namespace planning
}  // namespace apollo
