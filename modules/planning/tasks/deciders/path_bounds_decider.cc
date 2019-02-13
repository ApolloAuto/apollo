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
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

constexpr double kPathBoundsDeciderHorizon = 100.0;
constexpr double kPathBoundsDeciderResolution = 0.1;
constexpr double kDefaultLaneWidth = 5.0;
constexpr double kDefaultRoadWidth = 20.0;
constexpr double kRoadEdgeBuffer = 0.2;

PathBoundsDecider::PathBoundsDecider(const TaskConfig &config)
    : Decider(config) {}

Status PathBoundsDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // The decided path bounds should be in the format of: (s, l_min, l_max).
  std::vector<std::tuple<double, double, double>> path_boundaries;

  // 0. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundaries(
          reference_line_info->reference_line(),
          frame->PlanningStartPoint(), &path_boundaries)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // 1. Decide a rough boundary based on road info and ADC's position
  if (!GetBoundariesFromRoadsAndADC(
          reference_line_info->reference_line(),
          reference_line_info->AdcSlBoundary(), &path_boundaries)) {
    const std::string msg = "Failed to decide a rough boundary based on "
                            "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // 2. Finetune the boundary based on static obstacles
  // TODO(all): in the future, add side-pass functionality.
  if (!GetBoundariesFromStaticObstacles(
          reference_line_info->path_decision()->obstacles(),
          &path_boundaries)) {
    const std::string msg = "Failed to decide fine tune the boundaries after "
                            "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // Update the path boundary info to the frame.
  // TODO(all): update frame with path bound info.
  return Status::OK();
}

bool PathBoundsDecider::InitPathBoundaries(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& planning_start_point,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  // Sanity checks.
  CHECK_NOTNULL(path_boundaries);
  path_boundaries->clear();

  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  common::FrenetFramePoint adc_frenet_position = reference_line.GetFrenetPoint(
      planning_start_point.path_point());
  for (double curr_s = adc_frenet_position.s();
       curr_s < std::min(adc_frenet_position.s() + kPathBoundsDeciderHorizon,
                         reference_line.Length());
       curr_s += kPathBoundsDeciderResolution) {
    path_boundaries->emplace_back(
        curr_s,
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::max());
  }

  return true;
}

bool PathBoundsDecider::GetBoundariesFromRoadsAndADC(
    const ReferenceLine& reference_line,
    const SLBoundary& adc_sl_boundary,
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
  for (size_t i = 0; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Get the lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(
            curr_s, &curr_lane_left_width, &curr_lane_right_width)) {
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
    if (!reference_line.GetRoadWidth(
            curr_s, &curr_road_left_width, &curr_road_right_width)) {
      AWARN << "Failed to get road width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    // Calculate the proper boundary based on lane-width, road-width, and
    // ADC's position.
    double curr_left_bound =
        std::fmax(-curr_road_left_width,
                  std::fmin(-curr_lane_left_width, adc_sl_boundary.start_l()));
    double curr_right_bound =
        std::fmin(curr_road_right_width,
                  std::fmax(curr_lane_right_width, adc_sl_boundary.end_l()));
    // Update the boundary.
    std::get<1>((*path_boundaries)[i]) =
        std::fmax(std::get<1>((*path_boundaries)[i]),
                  curr_left_bound + GetBufferBetweenADCCenterAndEdge());
    std::get<2>((*path_boundaries)[i]) =
        std::fmin(std::get<2>((*path_boundaries)[i]),
                  curr_right_bound - GetBufferBetweenADCCenterAndEdge());
  }

  return true;
}

bool PathBoundsDecider::GetBoundariesFromStaticObstacles(
    const IndexedList<std::string, Obstacle>& indexed_obstacles,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {

  return true;
}

double PathBoundsDecider::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  // TODO(all): currently it's a fixed number. But it can take into account many
  // factors such as: ADC length, possible turning angle, speed, etc.
  constexpr double kAdcEdgeBuffer = 0.5;

  return (adc_half_width + kAdcEdgeBuffer);
}

}  // namespace planning
}  // namespace apollo
