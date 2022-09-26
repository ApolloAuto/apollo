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
 *   @file
 **/

#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/common/history.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/path/path_data.h"
#include "modules/planning/common/path_decision.h"
#include "modules/planning/common/speed/st_boundary.h"
#include "modules/planning/common/speed_limit.h"
#include "modules/common_msgs/planning_msgs/decision.pb.h"
#include "modules/planning/reference_line/reference_line.h"

namespace apollo {
namespace planning {

constexpr double kADCSafetyLBuffer = 0.1;
constexpr double kSIgnoreThreshold = 0.01;
constexpr double kTIgnoreThreshold = 0.1;
constexpr double kOvertakenObsCautionTime = 0.5;

class STObstaclesProcessor {
 public:
  STObstaclesProcessor() {}

  void Init(const double planning_distance, const double planning_time,
            const PathData& path_data, PathDecision* const path_decision,
            History* const history);

  virtual ~STObstaclesProcessor() = default;

  common::Status MapObstaclesToSTBoundaries(PathDecision* const path_decision);

  std::unordered_map<std::string, STBoundary> GetAllSTBoundaries();

  /** @brief Given a time t, get the lower and upper s-boundaries.
   * If the boundary is well-defined based on decision made previously,
   * fill "available_s_bounds" with only one boundary.
   * Otherwise, fill "available_s_bounds with all candidates and
   * "available_obs_decisions" with corresponding possible obstacle decisions.
   * @param Time t
   * @param The available s-boundaries to be filled up.
   * @param The corresponding possible obstacle decisions.
   * @return Whether we can get valid s-bounds.
   */
  bool GetSBoundsFromDecisions(
      double t,
      std::vector<std::pair<double, double>>* const available_s_bounds,
      std::vector<
          std::vector<std::pair<std::string, ObjectDecisionType>>>* const
          available_obs_decisions);

  /** @brief Provided that decisions for all existing obstacles are made, get
   * the speed limiting info from limiting st-obstacles.
   * @param Time t.
   * @param The actual limiting speed-info: (lower, upper)
   * @return True if there is speed limiting info; otherwise, false.
   */
  bool GetLimitingSpeedInfo(
      double t, std::pair<double, double>* const limiting_speed_info);

  /** @brief Set the decision for a given obstacle.
   */
  void SetObstacleDecision(const std::string& obs_id,
                           const ObjectDecisionType& obs_decision);

  /** @brief Set the decision for a list of obstacles.
   */
  void SetObstacleDecision(
      const std::vector<std::pair<std::string, ObjectDecisionType>>&
          obstacle_decisions);

 private:
  /** @brief Given a single obstacle, compute its ST-boundary.
   * @param An obstacle (if moving, should contain predicted trajectory).
   * @param A vector to be filled with lower edge of ST-polygon.
   * @param A vector to be filled with upper edge of ST-polygon.
   * @return If appears on ST-graph, return true; otherwise, false.
   */
  bool ComputeObstacleSTBoundary(const Obstacle& obstacle,
                                 std::vector<STPoint>* const lower_points,
                                 std::vector<STPoint>* const upper_points,
                                 bool* const is_caution_obstacle,
                                 double* const obs_caution_end_t);

  /** @brief Given ADC's path and an obstacle instance at a certain timestep,
   * get the upper and lower s that ADC might overlap with the obs instance.
   * @param A vector of ADC planned path points.
   * @param A obstacle at a certain timestep.
   * @param ADC lateral buffer for safety consideration.
   * @param The overlapping upper and lower s to be updated.
   * @return Whether there is an overlap or not.
   */
  bool GetOverlappingS(const std::vector<common::PathPoint>& adc_path_points,
                       const common::math::Box2d& obstacle_instance,
                       const double adc_l_buffer,
                       std::pair<double, double>* const overlapping_s);

  /** @brief Over the s-dimension, find the last point that is before the
   * obstacle instance of the first point that is after the obstacle.
   * If there exists no such point within the given range, return -1.
   * @param ADC path points
   * @param The obstacle box
   * @param The s threshold, must be non-negative.
   * @param The direction
   * @param The start-idx
   * @param The end-idx
   * @return Whether there is overlapping or not.
   */
  int GetSBoundingPathPointIndex(
      const std::vector<common::PathPoint>& adc_path_points,
      const common::math::Box2d& obstacle_instance, const double s_thresh,
      const bool is_before, const int start_idx, const int end_idx);

  /** @brief Over the s-dimension, check if the path-point is away
   * from the projected obstacle in the given direction.
   * @param A certain path-point.
   * @param The next path-point indicating path direction.
   * @param The obstacle bounding box.
   * @param The threshold s to tell if path point is far away.
   * @param Direction indicator. True if we want the path-point to be
   *        before the obstacle.
   * @return whether the path-point is away in the indicated direction.
   */
  bool IsPathPointAwayFromObstacle(const common::PathPoint& path_point,
                                   const common::PathPoint& direction_point,
                                   const common::math::Box2d& obs_box,
                                   const double s_thresh, const bool is_before);

  /** @brief Check if ADC is overlapping with the given obstacle box.
   * @param ADC's position.
   * @param Obstacle's box.
   * @param ADC's lateral buffer.
   * @return Whether ADC at that position is overlapping with the given
   * obstacle box.
   */
  bool IsADCOverlappingWithObstacle(const common::PathPoint& adc_path_point,
                                    const common::math::Box2d& obs_box,
                                    const double l_buffer) const;

  /** @brief Find the vertical (s) gaps of the st-graph.
   * @param Vector of obstacle-t-edges
   * @param The existing minimum s edge.
   * @param The existing maximum s edge.
   * @return A list of available s gaps for ADC to go.
   */
  std::vector<std::pair<double, double>> FindSGaps(
      const std::vector<std::tuple<int, double, double, double, std::string>>&
          obstacle_t_edges,
      double s_min, double s_max);

  /** @brief Based on obstacle position and prospective ADC position,
   * determine the obstacle decision.
   * @param Obstacle's minimum s.
   * @param Obstacle's maximum s.
   * @param ADC's prospective position.
   * @return The decision for the given obstacle.
   */
  ObjectDecisionType DetermineObstacleDecision(const double obs_s_min,
                                               const double obs_s_max,
                                               const double s) const;

  /** @brief Check if a given s falls within adc's low road right segment.
   * @param A certain S.
   * @return True if within; false otherwise.
   */
  bool IsSWithinADCLowRoadRightSegment(const double s) const;

 private:
  double planning_time_;
  double planning_distance_;
  PathData path_data_;
  common::VehicleParam vehicle_param_;
  double adc_path_init_s_;
  PathDecision* path_decision_;

  // A vector of sorted obstacle's t-edges:
  //  (is_starting_t, t, s_min, s_max, obs_id).
  std::vector<std::tuple<int, double, double, double, std::string>>
      obs_t_edges_;
  int obs_t_edges_idx_;

  std::unordered_map<std::string, STBoundary> obs_id_to_st_boundary_;
  std::unordered_map<std::string, ObjectDecisionType> obs_id_to_decision_;

  std::vector<std::tuple<std::string, STBoundary, Obstacle*>>
      candidate_clear_zones_;

  std::unordered_map<std::string, STBoundary>
      obs_id_to_alternative_st_boundary_;

  std::vector<std::pair<double, double>> adc_low_road_right_segments_;

  History* history_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
