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
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/tasks/deciders/path_decider_obstacle_utils.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;

// PathBoundPoint contains: (s, l_min, l_max).
using PathBoundPoint = std::tuple<double, double, double>;
// PathBoundary contains a vector of PathBoundPoints.
using PathBoundary = std::vector<PathBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id).
using ObstacleEdge = std::tuple<int, double, double, double, std::string>;

constexpr double kPathBoundsDeciderHorizon = 100.0;
constexpr double kPathBoundsDeciderResolution = 0.5;
constexpr double kDefaultLaneWidth = 5.0;
constexpr double kDefaultRoadWidth = 20.0;
constexpr double kObstacleSBuffer = 1.0;
constexpr double kObstacleLBuffer = 0.4;

PathBoundsDecider::PathBoundsDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Initialize.
  InitPathBoundsDecider(*frame, *reference_line_info);

  // The decided path bounds should be in the format of: (s, l_min, l_max).
  PathBoundary fallback_path_boundaries;
  PathBoundary path_boundaries;

  // Generate fallback path boundaries.
  std::string fallback_path_bounds_msg = GenerateFallbackPathBoundary(
      frame, reference_line_info, &fallback_path_boundaries);
  if (fallback_path_bounds_msg != "") {
    return Status(ErrorCode::PLANNING_ERROR, fallback_path_bounds_msg);
  }
  // Update the fallback path boundary into the reference_line_info.
  if (fallback_path_boundaries.empty()) {
    const std::string msg = "Failed to get a valid fallback path boundary";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  std::vector<std::pair<double, double>> fallback_path_boundaries_pair;
  for (size_t i = 0; i < fallback_path_boundaries.size(); ++i) {
    fallback_path_boundaries_pair.emplace_back(
        std::get<1>(fallback_path_boundaries[i]),
        std::get<2>(fallback_path_boundaries[i]));
  }
  reference_line_info->SetFallbackPathBoundaries(
      fallback_path_boundaries_pair, std::get<0>(fallback_path_boundaries[0]),
      kPathBoundsDeciderResolution);
  if (!fallback_path_boundaries.empty()) {
    CHECK_LE(adc_frenet_l_, std::get<2>(fallback_path_boundaries[0]));
    CHECK_GE(adc_frenet_l_, std::get<1>(fallback_path_boundaries[0]));
  }

  // Generate path boundaries.
  std::string path_bounds_msg =
      GenerateRegularPathBoundary(*frame, *reference_line_info,
                                  LaneBorrowInfo::NO_BORROW, &path_boundaries);
  if (path_bounds_msg != "") {
    return Status(ErrorCode::PLANNING_ERROR, path_bounds_msg);
  }
  // Update the path boundary into the reference_line_info.
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

  // Success
  ADEBUG << "Completed regular and fallback path boundaries generation.";
  return Status::OK();
}

void PathBoundsDecider::InitPathBoundsDecider(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const common::TrajectoryPoint& planning_start_point =
      frame.PlanningStartPoint();
  // Reset variables.
  blocking_obstacle_id_ = "";
  adc_frenet_s_ = 0.0;
  adc_frenet_l_ = 0.0;
  adc_lane_width_ = 0.0;

  // Initialize some private variables.
  // ADC s/l info.
  auto adc_frenet_position =
      reference_line.GetFrenetPoint(planning_start_point.path_point());
  adc_frenet_s_ = adc_frenet_position.s();
  adc_frenet_l_ = adc_frenet_position.l();
  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  adc_frenet_sd_ = adc_sl_info.first[1];
  adc_frenet_ld_ = adc_sl_info.second[1] * adc_frenet_sd_;
  // ADC's lane width.
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!reference_line.GetLaneWidth(adc_frenet_s_, &lane_left_width,
                                   &lane_right_width)) {
    AWARN << "Failed to get lane width at planning start point.";
    adc_lane_width_ = kDefaultLaneWidth;
  } else {
    adc_lane_width_ = lane_left_width + lane_right_width;
  }
}

std::string PathBoundsDecider::GenerateRegularPathBoundary(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const LaneBorrowInfo lane_borrow_info, PathBoundary* const path_boundary) {
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info.reference_line(), path_boundary)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return msg;
  }
  // PathBoundsDebugString(*path_boundary);

  // 2. Decide a rough boundary based on road info and ADC's position
  if (!GetBoundaryFromLanesAndADC(reference_line_info.reference_line(),
                                  lane_borrow_info, 0.1, path_boundary)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return msg;
  }
  // PathBoundsDebugString(*path_boundary);

  // 3. Fine-tune the boundary based on static obstacles
  if (!GetBoundaryFromStaticObstacles(reference_line_info.path_decision(),
                                      path_boundary)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return msg;
  }
  // PathBoundsDebugString(*path_boundary);

  // 4. Adjust the boundary considering dynamic obstacles
  // TODO(all): may need to implement this in the future.

  ADEBUG << "Completed generating path boundaries.";
  return "";
}

std::string PathBoundsDecider::GenerateFallbackPathBoundary(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    PathBoundary* const path_boundaries) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(reference_line_info->reference_line(),
                        path_boundaries)) {
    const std::string msg = "Failed to initialize fallback path boundaries.";
    AERROR << msg;
    return msg;
  }
  // PathBoundsDebugString(*path_boundaries);

  // 2. Decide a rough boundary based on road info and ADC's position
  if (!GetBoundaryFromLanesAndADC(reference_line_info->reference_line(),
                                  LaneBorrowInfo::NO_BORROW, 0.5,
                                  path_boundaries)) {
    const std::string msg =
        "Failed to decide a rough fallback boundary based on "
        "road information.";
    AERROR << msg;
    return msg;
  }
  // PathBoundsDebugString(*path_boundaries);

  ADEBUG << "Completed generating fallback path boundaries.";
  return "";
}

bool PathBoundsDecider::InitPathBoundary(const ReferenceLine& reference_line,
                                         PathBoundary* const path_boundary) {
  // Sanity checks.
  CHECK_NOTNULL(path_boundary);
  path_boundary->clear();

  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  for (double curr_s = adc_frenet_s_;
       curr_s < std::min(adc_frenet_s_ + kPathBoundsDeciderHorizon,
                         reference_line.Length());
       curr_s += kPathBoundsDeciderResolution) {
    path_boundary->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                                std::numeric_limits<double>::max());
  }

  // return.
  if (path_boundary->empty()) {
    ADEBUG << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}

bool PathBoundsDecider::GetBoundaryFromLanesAndADC(
    const ReferenceLine& reference_line, const LaneBorrowInfo lane_borrow_info,
    double ADC_buffer,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  // Sanity checks.
  CHECK_NOTNULL(path_boundaries);
  CHECK(!path_boundaries->empty());

  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_lane_width_ / 2.0;
  double past_lane_right_width = adc_lane_width_ / 2.0;
  double past_neighbor_lane_width = 0.0;

  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // 1. Get the current lane width at current point.
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

    // 2. Get the neighbor lane widths at the current point.
    double curr_neighbor_lane_width = 0.0;
    hdmap::Lane curr_lane;
    hdmap::LaneInfoConstPtr lane_info_ptr;
    if (!GetLaneInfoFromPoint(
            reference_line.GetReferencePoint(curr_s).x(),
            reference_line.GetReferencePoint(curr_s).y(), 0.0,
            reference_line.GetReferencePoint(curr_s).heading(),
            &lane_info_ptr)) {
      ADEBUG << "Cannot find the true current lane; therefore, use the "
                "planning starting point's lane as a substitute.";
      curr_neighbor_lane_width = past_neighbor_lane_width;
    } else {
      curr_lane = lane_info_ptr->lane();
      hdmap::LaneInfoConstPtr adjacent_lane = nullptr;
      if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
        // Borrowing left neighbor lane.
        if (curr_lane.left_neighbor_forward_lane_id_size() > 0) {
          adjacent_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
              curr_lane.left_neighbor_forward_lane_id(0));

        } else if (curr_lane.left_neighbor_reverse_lane_id_size() > 0) {
          adjacent_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
              curr_lane.left_neighbor_reverse_lane_id(0));
        }
      } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
        // Borrowing right neighbor lane.
        if (curr_lane.right_neighbor_forward_lane_id_size() > 0) {
          adjacent_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
              curr_lane.right_neighbor_forward_lane_id(0));
        } else if (curr_lane.right_neighbor_reverse_lane_id_size() > 0) {
          adjacent_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
              curr_lane.right_neighbor_reverse_lane_id(0));
        }
      }
      common::math::Vec2d xy_curr_s;
      common::SLPoint sl_curr_s;
      sl_curr_s.set_s(curr_s);
      sl_curr_s.set_l(0.0);
      reference_line.SLToXY(sl_curr_s, &xy_curr_s);
      double adjacent_lane_s = 0.0;
      double adjacent_lane_l = 0.0;
      if (adjacent_lane == nullptr ||
          !adjacent_lane->GetProjection(xy_curr_s, &adjacent_lane_s,
                                        &adjacent_lane_l)) {
        ADEBUG << "Unable to get the neighbor lane's width.";
        curr_neighbor_lane_width = past_neighbor_lane_width;
      } else {
        curr_neighbor_lane_width = adjacent_lane->GetWidth(adjacent_lane_s);
        past_neighbor_lane_width = curr_neighbor_lane_width;
      }
    }

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    constexpr double kMaxLateralAccelerations = 1.0;
    double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                              adc_frenet_ld_ * adc_frenet_ld_ /
                              kMaxLateralAccelerations / 2.0;

    double curr_left_bound_lane =
        curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);
    double curr_left_bound_adc =
        std::fmax(adc_frenet_l_, adc_frenet_l_ + ADC_speed_buffer) +
        GetBufferBetweenADCCenterAndEdge() + ADC_buffer;
    double curr_left_bound =
        std::fmax(curr_left_bound_lane, curr_left_bound_adc);

    double curr_right_bound_lane =
        -curr_lane_right_width -
        (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
             ? curr_neighbor_lane_width
             : 0.0);
    double curr_right_bound_adc =
        std::fmin(adc_frenet_l_, adc_frenet_l_ + ADC_speed_buffer) -
        GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
    double curr_right_bound =
        std::fmin(curr_right_bound_lane, curr_right_bound_adc);

    // 4. Update the boundary.
    double dummy = 0.0;
    if (!UpdatePathBoundaryAndCenterLine(i, curr_left_bound, curr_right_bound,
                                         path_boundaries, &dummy)) {
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
bool PathBoundsDecider::GetBoundaryFromStaticObstacles(
    const PathDecision& path_decision,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  // Preprocessing.
  auto indexed_obstacles = path_decision.obstacles();
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

bool PathBoundsDecider::GetLaneInfoFromPoint(
    double point_x, double point_y, double point_z, double point_theta,
    hdmap::LaneInfoConstPtr* const lane) {
  constexpr double kLaneSearchRadius = 1.0;
  constexpr double kLaneSearchMaxThetaDiff = M_PI / 3.0;
  double s = 0.0;
  double l = 0.0;
  if (HDMapUtil::BaseMapPtr()->GetNearestLaneWithHeading(
          common::util::MakePointENU(point_x, point_y, point_z),
          kLaneSearchRadius, point_theta, kLaneSearchMaxThetaDiff, lane, &s,
          &l) != 0) {
    AWARN << "Failed to find nearest lane from map at position: "
          << "(x, y, z) = (" << point_x << ", " << point_y << ", " << point_z
          << ")"
          << ", heading = " << point_theta;
    return false;
  }
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
std::vector<ObstacleEdge> PathBoundsDecider::SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles) {
  std::vector<ObstacleEdge> sorted_obstacles;

  // Go through every obstacle and preprocess it.
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Only focus on those within-scope obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Only focus on obstacles that are ahead of ADC.
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
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return sorted_obstacles;
}

std::vector<PathBoundary> PathBoundsDecider::ConstructSubsequentPathBounds(
    const std::vector<ObstacleEdge>& sorted_obstacles, size_t path_idx,
    size_t obs_idx,
    std::unordered_map<std::string, std::tuple<bool, double>>* const
        obs_id_to_details,
    PathBoundary* const curr_path_bounds) {
  double left_bounds_from_obstacles = std::numeric_limits<double>::max();
  double right_bounds_from_obstacles = std::numeric_limits<double>::lowest();
  double curr_s = std::get<0>((*curr_path_bounds)[path_idx]);
  //==============================================================
  // If searched through all available s and found a path, return.
  if (path_idx >= curr_path_bounds->size()) {
    ADEBUG << "Completed path bounds search ending at path_idx = " << path_idx;
    return {*curr_path_bounds};
  }

  //==============================================================
  // If there is no obstacle updates at this path_idx.
  if (obs_idx >= sorted_obstacles.size() ||
      std::get<1>(sorted_obstacles[obs_idx]) > curr_s) {
    // 0. Backup the old memory.
    double dummy = 0.0;
    auto old_path_boundary = *curr_path_bounds;
    // 1. Get the boundary from obstacles.
    for (auto it = obs_id_to_details->begin(); it != obs_id_to_details->end();
         ++it) {
      if (std::get<0>(it->second)) {
        // Pass from left.
        right_bounds_from_obstacles =
            std::max(right_bounds_from_obstacles, std::get<1>(it->second));
      } else {
        // Pass from right.
        left_bounds_from_obstacles =
            std::min(left_bounds_from_obstacles, std::get<1>(it->second));
      }
    }
    // 2. Update the path boundary
    bool is_able_to_update = UpdatePathBoundaryAndCenterLine(
        path_idx, left_bounds_from_obstacles, right_bounds_from_obstacles,
        curr_path_bounds, &dummy);
    // 3. Return proper values.
    std::vector<PathBoundary> ret;
    if (is_able_to_update) {
      ret =
          ConstructSubsequentPathBounds(sorted_obstacles, path_idx + 1, obs_idx,
                                        obs_id_to_details, curr_path_bounds);
    } else {
      TrimPathBounds(static_cast<int>(path_idx), curr_path_bounds);
      ret.push_back(*curr_path_bounds);
    }
    *curr_path_bounds = old_path_boundary;
    return ret;
  }

  //==============================================================
  // If there are obstacle changes
  // 0. Backup the old memory.
  std::unordered_map<std::string, std::tuple<bool, double>>
      old_obs_id_to_details = *obs_id_to_details;
  auto old_path_boundary = *curr_path_bounds;
  double dummy = 0.0;
  // 1. Go through all obstacle changes.
  //    - For exiting obstacle, remove from our memory.
  //    - For entering obstalce, save it to a vector.
  std::vector<ObstacleEdge> new_entering_obstacles;
  size_t new_obs_idx = obs_idx;
  while (new_obs_idx < sorted_obstacles.size() &&
         std::get<1>(sorted_obstacles[new_obs_idx]) <= curr_s) {
    if (!std::get<0>(sorted_obstacles[new_obs_idx])) {
      // For exiting obstacle.
      obs_id_to_details->erase(std::get<4>(sorted_obstacles[new_obs_idx]));
    } else {
      // For entering obstacle.
      new_entering_obstacles.push_back(sorted_obstacles[new_obs_idx]);
    }
    ++new_obs_idx;
  }
  // 2. For new entering obstacles, decide possible pass directions.
  //    (ranked in terms of optimality)
  auto pass_direction_decisions =
      DecidePassDirections(0.0, 0.0, new_entering_obstacles);
  // 3. Try constructing subsequent path-bounds for all possible directions.
  std::vector<PathBoundary> ret;
  for (size_t i = 0; i < pass_direction_decisions.size(); ++i) {
    // For each possible direction:
    // a. Update the obs_id_to_details
    for (size_t j = 0; j < pass_direction_decisions[i].size(); ++j) {
      if (pass_direction_decisions[i][j]) {
        // Pass from left.
        (*obs_id_to_details)[std::get<4>(new_entering_obstacles[j])] =
            std::make_tuple(true, std::get<3>(new_entering_obstacles[j]));
      } else {
        // Pass from right.
        (*obs_id_to_details)[std::get<4>(new_entering_obstacles[j])] =
            std::make_tuple(false, std::get<2>(new_entering_obstacles[j]));
      }
    }
    // b. Figure out left/right bounds after the updates.
    for (auto it = obs_id_to_details->begin(); it != obs_id_to_details->end();
         ++it) {
      if (std::get<0>(it->second)) {
        // Pass from left.
        right_bounds_from_obstacles =
            std::max(right_bounds_from_obstacles, std::get<1>(it->second));
      } else {
        // Pass from right.
        left_bounds_from_obstacles =
            std::min(left_bounds_from_obstacles, std::get<1>(it->second));
      }
    }
    // c. Update for this path_idx, and construct the subsequent path bounds.
    std::vector<PathBoundary> curr_dir_path_boundaries;
    bool is_able_to_update = UpdatePathBoundaryAndCenterLine(
        path_idx, left_bounds_from_obstacles, right_bounds_from_obstacles,
        curr_path_bounds, &dummy);
    if (is_able_to_update) {
      curr_dir_path_boundaries = ConstructSubsequentPathBounds(
          sorted_obstacles, path_idx + 1, new_obs_idx, obs_id_to_details,
          curr_path_bounds);
    } else {
      TrimPathBounds(static_cast<int>(path_idx), curr_path_bounds);
      curr_dir_path_boundaries.push_back(*curr_path_bounds);
    }
    // d. Update the path_bounds into the vector, and revert changes
    //    to curr_path_bounds for next cycle.
    ret.insert(ret.end(), curr_dir_path_boundaries.begin(),
               curr_dir_path_boundaries.end());
    *curr_path_bounds = old_path_boundary;
  }
  // 4. Select the best path_bounds in ret.
  *obs_id_to_details = old_obs_id_to_details;
  *curr_path_bounds = old_path_boundary;
  std::sort(ret.begin(), ret.end(),
            [](const PathBoundary& lhs, const PathBoundary& rhs) {
              return lhs.size() > rhs.size();
            });
  while (ret.size() > 3) {
    ret.pop_back();
  }
  return ret;
}

std::vector<std::vector<bool>> PathBoundsDecider::DecidePassDirections(
    double l_min, double l_max,
    const std::vector<ObstacleEdge>& new_entering_obstacles) {
  std::vector<std::vector<bool>> decisions;

  // Convert into lateral edges.
  std::vector<ObstacleEdge> lateral_edges;
  lateral_edges.emplace_back(1, std::numeric_limits<double>::lowest(), 0.0, 0.0,
                             "l_min");
  lateral_edges.emplace_back(0, l_min, 0.0, 0.0, "l_min");
  lateral_edges.emplace_back(1, l_max, 0.0, 0.0, "l_max");
  lateral_edges.emplace_back(0, std::numeric_limits<double>::max(), 0.0, 0.0,
                             "l_max");
  for (size_t i = 0; i < new_entering_obstacles.size(); ++i) {
    if (std::get<3>(new_entering_obstacles[i]) < l_min ||
        std::get<2>(new_entering_obstacles[i]) > l_max) {
      continue;
    }
    lateral_edges.emplace_back(1, std::get<2>(new_entering_obstacles[i]), 0.0,
                               0.0, std::get<4>(new_entering_obstacles[i]));
    lateral_edges.emplace_back(0, std::get<3>(new_entering_obstacles[i]), 0.0,
                               0.0, std::get<4>(new_entering_obstacles[i]));
  }
  // Sort the lateral edges for lateral sweep-line algorithm.
  std::sort(lateral_edges.begin(), lateral_edges.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  // Go through the lateral edges and find any possible slot.
  std::vector<double> empty_slot;
  int num_obs = 0;
  for (size_t i = 0; i < lateral_edges.size(); ++i) {
    // Update obstacle overlapping info.
    if (std::get<0>(lateral_edges[i])) {
      ++num_obs;
    } else {
      --num_obs;
    }
    // If there is an empty slot within lane boundary.
    if (num_obs == 0 && i != lateral_edges.size() - 1) {
      empty_slot.push_back(
          (std::get<1>(lateral_edges[i]) + std::get<1>(lateral_edges[i + 1])) /
          2.0);
    }
  }
  // For each empty slot, update a corresponding pass direction
  for (size_t i = 0; i < empty_slot.size(); ++i) {
    double pass_position = empty_slot[i];
    std::vector<bool> pass_direction;
    for (size_t j = 0; j < new_entering_obstacles.size(); ++j) {
      if (std::get<2>(new_entering_obstacles[j]) > pass_position) {
        pass_direction.push_back(false);
      } else {
        pass_direction.push_back(true);
      }
    }
    decisions.push_back(pass_direction);
  }
  // TODO(jiacheng): sort the decisions based on the feasibility.

  return decisions;
}

bool PathBoundsDecider::UpdatePathBoundaryAndCenterLine(
    size_t idx, double left_bound, double right_bound,
    PathBoundary* const path_boundaries, double* const center_line) {
  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(std::get<2>((*path_boundaries)[idx]),
                               left_bound - GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2.0;
  return true;
}

void PathBoundsDecider::TrimPathBounds(const int path_blocked_idx,
                                       PathBoundary* const path_boundaries) {
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
    const PathBoundary& path_boundaries) {
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    ADEBUG << "idx " << i << "; s = " << std::get<0>(path_boundaries[i])
           << "; l_min = " << std::get<1>(path_boundaries[i])
           << "; l_max = " << std::get<2>(path_boundaries[i]);
  }
}

}  // namespace planning
}  // namespace apollo
