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

#include "modules/planning/tasks/deciders/path_bounds_decider.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

constexpr double kPathBoundsDeciderHorizon = 100.0;
constexpr double kPathBoundsDeciderResolution = 0.5;
constexpr double kDefaultLaneWidth = 5.0;
constexpr double kDefaultRoadWidth = 20.0;
constexpr double kObstacleSBuffer = 1.0;
constexpr double kObstacleLBuffer = 0.3;

PathBoundsDecider::PathBoundsDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // The decided path bounds should be in the format of: (s, l_min, l_max).
  std::vector<std::tuple<double, double, double>> path_boundaries;

  // 0. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundaries(reference_line_info->reference_line(),
                          frame->PlanningStartPoint(), &path_boundaries)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(path_boundaries);

  // 1. Decide a rough boundary based on road info and ADC's position
  if (!GetBoundariesFromRoadsAndADC(reference_line_info->reference_line(),
                                    reference_line_info->AdcSlBoundary(),
                                    &path_boundaries)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(path_boundaries);

  // 2. Fine-tune the boundary based on static obstacles
  // TODO(all): in the future, add side-pass functionality.
  if (!GetBoundariesFromStaticObstacles(
          reference_line_info->path_decision(),
          &path_boundaries)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // PathBoundsDebugString(path_boundaries);

  // 3. Adjust the boundary considering dynamic obstacles
  // TODO(all): may need to implement this in the future.

  // Update the path boundary info to the frame.
  if (path_boundaries.empty()) {
    const std::string msg = "Failed to get a valid path boundary";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  std::vector<std::pair<double, double>> path_boundaries_pair;
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    path_boundaries_pair.emplace_back(std::get<1>(path_boundaries[i]),
                                      std::get<2>(path_boundaries[i]));
  }
  reference_line_info->SetPathBoundaries(path_boundaries_pair,
                                         std::get<0>(path_boundaries[0]),
                                         kPathBoundsDeciderResolution);
  reference_line_info->SetBlockingObstacleId(blocking_obstacle_id_);
  if (!path_boundaries.empty()) {
    CHECK_LE(adc_frenet_l_, std::get<2>(path_boundaries[0]));
    CHECK_GE(adc_frenet_l_, std::get<1>(path_boundaries[0]));
  }
  ADEBUG << "Completed path boundaries generation.";
  return Status::OK();
}

bool PathBoundsDecider::InitPathBoundaries(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& planning_start_point,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  // Sanity checks.
  CHECK_NOTNULL(path_boundaries);
  path_boundaries->clear();

  // Reset variables
  blocking_obstacle_id_ = "";
  adc_frenet_s_ = 0.0;
  adc_frenet_l_ = 0.0;

  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  auto adc_frenet_position =
      reference_line.GetFrenetPoint(planning_start_point.path_point());
  adc_frenet_s_ = adc_frenet_position.s();
  adc_frenet_l_ = adc_frenet_position.l();
  for (double curr_s = adc_frenet_s_;
       curr_s < std::min(adc_frenet_s_ + kPathBoundsDeciderHorizon,
                         reference_line.Length());
       curr_s += kPathBoundsDeciderResolution) {
    path_boundaries->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                                  std::numeric_limits<double>::max());
  }

  return true;
}

bool PathBoundsDecider::GetBoundariesFromRoadsAndADC(
    const ReferenceLine& reference_line, const SLBoundary& adc_sl_boundary,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  // Sanity checks.
  CHECK_NOTNULL(path_boundaries);
  CHECK(!path_boundaries->empty());

  // Go through every point, update the boundary based on road info and
  // ADC's position.
  double past_lane_left_width = kDefaultLaneWidth / 2.0;
  double past_lane_right_width = kDefaultLaneWidth / 2.0;
  double past_road_left_width = kDefaultRoadWidth / 2.0;
  double past_road_right_width = kDefaultRoadWidth / 2.0;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Get the lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }
    // Get the road width at current point.
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    if (!reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN << "Failed to get road width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    // Calculate the proper boundary based on lane-width, road-width, and
    // ADC's position.
    /*
    ADEBUG << "At s = " << std::get<0>((*path_boundaries)[i])
           << ", lane_left_W = " << curr_lane_left_width
           << ", lane_right_W = " << curr_lane_right_width
           << ", road_left_W = " << curr_road_left_width
           << ", road_right_W = " << curr_road_right_width
           << ", ADC W = " << GetBufferBetweenADCCenterAndEdge();
    */

    double curr_left_bound =
        std::fmax(curr_lane_left_width,
                  adc_frenet_l_ + GetBufferBetweenADCCenterAndEdge() + 0.1);
    double curr_right_bound =
        std::fmin(-curr_lane_right_width,
                  adc_frenet_l_ - GetBufferBetweenADCCenterAndEdge() - 0.1);
    // Update the boundary.
    double dummy = 0.0;
    if (!UpdatePathBoundaryAndCenterLine(
            i, curr_left_bound, curr_right_bound, path_boundaries, &dummy)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_boundaries);

  return true;
}


// Currently, it processes each obstacle based on its frenet-frame
// projection. Therefore, it might be overly conservative when processing
// obstacles whose headings differ from road-headings a lot.
// TODO(all): (future work) this can be improved in the future.
bool PathBoundsDecider::GetBoundariesFromStaticObstacles(
    PathDecision* const path_decision,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  // Preprocessing.
  auto indexed_obstacles = path_decision->obstacles();
  auto sorted_obstacles = SortObstaclesForSweepLine(indexed_obstacles);
  double center_line = adc_frenet_l_;
  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<double, std::greater<double>> right_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;
  left_bounds.insert(std::numeric_limits<double>::max());
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;
  // Maps obstacle ID's to the decision of whether side-pass on this obstacle
  // is allowed. If allowed, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;

  // Step through every path point.
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Check and see if there is any obstacle change:
    if (obs_idx < sorted_obstacles.size() &&
        std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
      while (obs_idx < sorted_obstacles.size() &&
             std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        std::string curr_obstacle_id = std::get<4>(curr_obstacle);
        if (std::get<0>(curr_obstacle) == 1) {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.
          // TODO(all): (future work) can make this DFS all possible
          // directions. (with proper early stopping mechanisms to save time)
          if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
            // Obstacle is to the right of center-line, should pass from left.
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);
            if (!UpdatePathBoundaryAndCenterLine(
                    i, *left_bounds.begin(), *right_bounds.begin(),
                    path_boundaries, &center_line)) {
              path_blocked_idx = static_cast<int>(i);
              blocking_obstacle_id_ = curr_obstacle_id;
              break;
              // ADC has no path, start side-pass decision.
              // 1. Don't side-pass if obstacle is blocked by other obstacles
              //    and is not a parked vehicle.
              //    Don't side-pass virtual/ped/cyclist obstacles.
              // TODO(all): finish this.
            }
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);
            if (!UpdatePathBoundaryAndCenterLine(
                    i, *left_bounds.begin(), *right_bounds.begin(),
                    path_boundaries, &center_line)) {
              path_blocked_idx = static_cast<int>(i);
              blocking_obstacle_id_ = curr_obstacle_id;
              break;
            }
          }
        } else {
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);
        }
        // Update the bounds and center_line.
        std::get<1>((*path_boundaries)[i]) = std::fmax(
            std::get<1>((*path_boundaries)[i]),
            *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
        std::get<2>((*path_boundaries)[i]) = std::fmin(
            std::get<2>((*path_boundaries)[i]),
            *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
        if (std::get<1>((*path_boundaries)[i]) >
            std::get<2>((*path_boundaries)[i])) {
          ADEBUG << "Path is blocked at s = " << curr_s;
          path_blocked_idx = static_cast<int>(i);
          // Currently, no side-pass when blocked.
          // TODO(all): implement the side-pass feature by considering
          // borrowing the adjacent lane.
          break;
        } else {
          center_line = (std::get<1>((*path_boundaries)[i]) +
                         std::get<2>((*path_boundaries)[i])) /
                        2.0;
        }

        ++obs_idx;
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
      std::get<1>((*path_boundaries)[i]) =
          std::fmax(std::get<1>((*path_boundaries)[i]),
                    *right_bounds.begin() + GetBufferBetweenADCCenterAndEdge());
      std::get<2>((*path_boundaries)[i]) =
          std::fmin(std::get<2>((*path_boundaries)[i]),
                    *left_bounds.begin() - GetBufferBetweenADCCenterAndEdge());
      if (std::get<1>((*path_boundaries)[i]) >
          std::get<2>((*path_boundaries)[i])) {
        ADEBUG << "Path is blocked at s = " << curr_s;
        path_blocked_idx = static_cast<int>(i);
        // Currently, no side-pass when blocked.
        // TODO(all): implement the side-pass feature by considering
        // borrowing the adjacent lane.
      } else {
        center_line = (std::get<1>((*path_boundaries)[i]) +
                       std::get<2>((*path_boundaries)[i])) /
                      2.0;
      }
    }

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_boundaries);

  return true;
}

double PathBoundsDecider::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  // TODO(all): currently it's a fixed number. But it can take into account many
  // factors such as: ADC length, possible turning angle, speed, etc.
  constexpr double kAdcEdgeBuffer = 0.0;

  return (adc_half_width + kAdcEdgeBuffer);
}

// The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
std::vector<std::tuple<int, double, double, double, std::string>>
PathBoundsDecider::SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles) {
  std::vector<std::tuple<int, double, double, double, std::string>>
      sorted_obstacles;

  // Go through every obstacle and preprocess it.
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Only focus on non-virtual obstacles.
    if (obstacle->IsVirtual()) {
      continue;
    }
    // Only focus on static obstacles.
    if (!obstacle->IsStatic()) {
      continue;
    }
    // Only focus on obstaclse that are ahead of ADC.
    if (obstacle->PerceptionSLBoundary().end_s() < adc_frenet_s_) {
      continue;
    }
    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    sorted_obstacles.emplace_back(1, obstacle_sl.start_s() - kObstacleSBuffer,
                                  obstacle_sl.start_l() - kObstacleLBuffer,
                                  obstacle_sl.end_l() + kObstacleLBuffer,
                                  obstacle->Id());
    sorted_obstacles.emplace_back(0, obstacle_sl.end_s() + kObstacleSBuffer,
                                  obstacle_sl.start_l() - kObstacleLBuffer,
                                  obstacle_sl.end_l() + kObstacleLBuffer,
                                  obstacle->Id());
  }

  // Sort.
  sort(sorted_obstacles.begin(), sorted_obstacles.end(),
       [](const std::tuple<int, double, double, double, std::string>& lhs,
          const std::tuple<int, double, double, double, std::string>& rhs) {
         if (std::get<1>(lhs) != std::get<1>(rhs)) {
           return std::get<1>(lhs) < std::get<1>(rhs);
         } else {
           return std::get<0>(lhs) < std::get<0>(rhs);
         }
       });

  return sorted_obstacles;
}

size_t PathBoundsDecider::ConstructSubsequentPathBounds(
    const std::vector<std::tuple<int, double, double, double, std::string>>&
    sorted_obstacles, size_t path_idx, size_t obs_idx,
    std::vector<std::tuple<double, double, double>>* const path_boundaries,
    std::multiset<double>* const left_bounds,
    std::multiset<double, std::greater<double>>* const right_bounds,
    double* const center_line,
    std::unordered_map<std::string, bool>* const obs_id_to_direction,
    std::unordered_map<std::string, bool>* const obs_id_to_sidepass_decision,
    std::vector<std::tuple<double, double, double>>* const final_path_bounds) {
  double curr_s = std::get<0>((*path_boundaries)[path_idx]);
  //==============================================================
  // If searched through all available s and found a path, return.
  if (path_idx >= path_boundaries->size()) {
    ADEBUG << "Completed path bounds search with path_idx = " << path_idx;
    *final_path_bounds = *path_boundaries;
    return path_idx;
  }

  //==============================================================
  // If there is no obstacle updates at this path_idx.
  if (obs_idx >= sorted_obstacles.size() ||
      std::get<1>(sorted_obstacles[obs_idx]) > curr_s) {
    ADEBUG << "There is no obstacle change at path_idx = " << path_idx;
    auto old_path_bound = (*path_boundaries)[path_idx];
    double old_center_line = *center_line;
    bool is_not_blocked = UpdatePathBoundaryAndCenterLine(
        path_idx, *left_bounds->begin(), *right_bounds->begin(),
        path_boundaries, center_line);
    size_t ret_val = 0;
    if (is_not_blocked) {
      ret_val = ConstructSubsequentPathBounds(
          sorted_obstacles, path_idx + 1, obs_idx,
          path_boundaries, left_bounds, right_bounds, center_line,
          obs_id_to_direction, obs_id_to_sidepass_decision,
          final_path_bounds);
    } else {
      ret_val = path_idx;
      *final_path_bounds = *path_boundaries;
    }
    (*path_boundaries)[path_idx] = old_path_bound;
    *center_line = old_center_line;
    return ret_val;
  }

  //==============================================================
  // If there is obstacle leaving


  //==============================================================
  // If there is obstacle entering



  //==============================================================
  // Do what needs to be done for the current path_idx and/or obs_idx.
  
  if (obs_idx < sorted_obstacles.size() &&
      std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
    // If there is some obstacle entering/exiting at this path_idx
    // Some preprocessing.
    const auto& curr_obstacle = sorted_obstacles[obs_idx];
    double curr_obstacle_l_min = std::get<2>(curr_obstacle);
    double curr_obstacle_l_max = std::get<3>(curr_obstacle);
    std::string curr_obstacle_id = std::get<4>(curr_obstacle);
    // Check if it's entering or exiting.
    if (std::get<0>(curr_obstacle) == 1) {
      // *** A new obstacle is entering ***
      // 1. Determine a better INITIAL direction for ADC to try passing.
      bool is_passing_from_left = true;
      if (curr_obstacle_l_min + curr_obstacle_l_max < *center_line * 2) {
        is_passing_from_left = true;
      } else {
        is_passing_from_left = false;
      }
      // 2. Try the determined direciton first, if failed, try the other one.
      for (int i = 0; i < 2; ++i) {
        // a. Update the bounds.
        if (is_passing_from_left) {
          (*obs_id_to_direction)[curr_obstacle_id] = true;
          right_bounds->insert(curr_obstacle_l_max);
        } else {
          (*obs_id_to_direction)[curr_obstacle_id] = false;
          left_bounds->insert(curr_obstacle_l_min);
        }
        // b. Check if able to proceed? If not, this direction doesn't work.
        // TODO(all): later, try borrowing neighbor lane before claiming this
        //            direction doesn't work.
        bool is_able_to_find_feasible_bound =
            UpdatePathBoundaryAndCenterLine(
                path_idx, *left_bounds->begin(), *right_bounds->begin(),
                path_boundaries, center_line);
        // c. If it's able to proceed, try constructing subsequent path_bounds.
        if (is_able_to_find_feasible_bound) {
          bool is_able_to_construct_subsequent_path =
              ConstructSubsequentPathBounds(
                  sorted_obstacles, path_idx, obs_idx + 1,
                  path_boundaries, left_bounds, right_bounds, center_line,
                  obs_id_to_direction, obs_id_to_sidepass_decision,
                  final_path_bounds);
          if (is_able_to_construct_subsequent_path) {
            return true;
          }
        }
        // d. If it turns out this direction doesn't work, revert any change
        // made before.
        obs_id_to_direction->erase(curr_obstacle_id);
        if (is_passing_from_left) {
          right_bounds->erase(curr_obstacle_l_max);
        } else {
          left_bounds->erase(curr_obstacle_l_min);
        }
        if (is_able_to_find_feasible_bound) {
          UpdatePathBoundaryAndCenterLine(
              path_idx, *left_bounds->begin(), *right_bounds->begin(),
              path_boundaries, center_line);
        }
        // e. Change side-pass direction and try again.
        is_passing_from_left = !is_passing_from_left;
        continue;
      }
      // 3. Failed for both directions. Unable to find a feasible path_boundary
      //    under the given conditions starting from path_idx.
      return false;
    } else {
      // *** An existing obstacle is exiting ***
      // UpdatePathBoundsForExitingObstacle();
    }
  } else {
    // *** If there is no obstacle change at this path_idx ***
    UpdatePathBoundaryAndCenterLine(
        path_idx, *left_bounds->begin(), *right_bounds->begin(),
        path_boundaries, center_line);
  }

  //==============================================================
  return true;
}

bool PathBoundsDecider::UpdatePathBoundaryAndCenterLine(
    size_t idx, double left_bound, double right_bound,
    std::vector<std::tuple<double, double, double>>* const path_boundaries,
    double* const center_line) {
  // Update the right bound (l_min):
  double new_l_min = std::fmax(
      std::get<1>((*path_boundaries)[idx]),
      right_bound + GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(
      std::get<2>((*path_boundaries)[idx]),
      left_bound - GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (std::get<1>((*path_boundaries)[idx]) >
      std::get<2>((*path_boundaries)[idx])) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) / 2.0;
  return true;
}

void PathBoundsDecider::TrimPathBounds(
    int path_blocked_idx,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

void PathBoundsDecider::PathBoundsDebugString(
      const std::vector<std::tuple<double, double, double>>& path_boundaries) {
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    ADEBUG << "idx " << i
           << "; s = " << std::get<0>(path_boundaries[i])
           << "; l_min = " << std::get<1>(path_boundaries[i])
           << "; l_max = " << std::get<2>(path_boundaries[i]);
  }
}

}  // namespace planning
}  // namespace apollo
