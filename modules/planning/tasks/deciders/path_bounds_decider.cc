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

#include <string>
#include <tuple>
#include <vector>

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

constexpr double kPathBoundsDeciderHorizon = 100.0;
constexpr double kPathBoundsDeciderResolution = 0.1;

PathBoundsDecider::PathBoundsDecider(const TaskConfig &config)
    : Decider(config) {}

Status PathBoundsDecider::Process(
    Frame *const frame, ReferenceLineInfo *const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // 0. Initialize the path boundaries to be an indefinitely large area.
  std::vector<std::tuple<double, double, double>> path_boundaries;
  if (!InitPathBoundaries(
          reference_line_info->reference_line(), &path_boundaries)) {
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
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  return true;
}

bool PathBoundsDecider::GetBoundariesFromRoadsAndADC(
    const ReferenceLine& reference_line,
    const SLBoundary& adc_sl_boundary,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  return true;
}

bool PathBoundsDecider::GetBoundariesFromStaticObstacles(
    const IndexedList<std::string, Obstacle>& indexed_obstacles,
    std::vector<std::tuple<double, double, double>>* const path_boundaries) {
  return true;
}


}  // namespace planning
}  // namespace apollo
