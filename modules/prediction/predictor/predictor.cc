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

#include "modules/prediction/predictor/predictor.h"

#include <vector>

namespace apollo {
namespace prediction {

const std::vector<Trajectory>& Predictor::trajectories() {
  return trajectories_;
}

int Predictor::NumOfTrajectories() { return trajectories_.size(); }

Trajectory Predictor::GenerateTrajectory(
    const std::vector<apollo::common::TrajectoryPoint>& points) {
  Trajectory trajectory;
  *trajectory.mutable_trajectory_point() = {points.begin(), points.end()};
  return trajectory;
}

void Predictor::SetEqualProbability(double probability, int start_index) {
  int num = NumOfTrajectories();
  if (start_index >= 0 && num > start_index) {
    probability /= static_cast<double>(num - start_index);
    for (int i = start_index; i < num; ++i) {
      trajectories_[i].set_probability(probability);
    }
  }
}

void Predictor::Clear() { trajectories_.clear(); }

}  // namespace prediction
}  // namespace apollo
