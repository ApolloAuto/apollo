/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
 * @file prediction_trajectory.cc
 **/

#include "modules/planning/common/trajectory/prediction_trajectory.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

double PredictionTrajectory::probability() const {
  return _probability;
}

double PredictionTrajectory::start_timestamp() const {
  return _start_timestamp;
}

void PredictionTrajectory::set_probability(const double prob) {
  _probability = prob;
}

void PredictionTrajectory::set_start_timestamp(const double ts) {
  _start_timestamp = ts;
}

TrajectoryPoint* PredictionTrajectory::trajectory_point_ptr(
    const std::uint32_t index) {
  if (index >= _trajectory_points.size()) {
    return nullptr;
  }
  return &_trajectory_points[index];
}

}  // namespace planning
}  // namespace apollo
