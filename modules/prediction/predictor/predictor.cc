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
#include <string>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"

namespace apollo {
namespace prediction {

using apollo::planning::ADCTrajectory;
using apollo::common::math::LineSegment2d;
using apollo::common::math::Vec2d;

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

void Predictor::TrimTrajectories(
    const ADCTrajectoryContainer* adc_trajectory_container) {
  for (size_t i = 0; i < trajectories_.size(); ++i) {
    TrimTrajectory(adc_trajectory_container, &trajectories_[i]);
  }
}

bool Predictor::TrimTrajectory(
    const ADCTrajectoryContainer* adc_trajectory_container,
    Trajectory* trajectory) {
  int num_point = trajectory->trajectory_point_size();
  // Get the index at intersect
  int index = 0;
  bool has_intersect = false;
  while (index < num_point) {
    double x = trajectory->trajectory_point(index).path_point().x();
    double y = trajectory->trajectory_point(index).path_point().y();
    std::vector<std::string> lane_ids =
        PredictionMap::instance()->NearbyLaneIds(x, y,
            FLAGS_distance_to_adc_trajectory_thred);
    for (const std::string& lane_id : lane_ids) {
      if (adc_trajectory_container->ContainsLaneId(lane_id)) {
        has_intersect = true;
        break;
      }
    }
    if (has_intersect) {
      break;
    }
    ++index;
  }

  // if no intersect
  if (index == num_point) {
    return false;
  }

  double index_time = trajectory->trajectory_point(index).relative_time();
  // if early intersect occurs
  if (index_time < FLAGS_time_to_adc_trajectory_thred) {
    return false;
  }

  for (int i = index; i < num_point; ++i) {
    trajectory->mutable_trajectory_point()->RemoveLast();
  }
  return true;
}

}  // namespace prediction
}  // namespace apollo
