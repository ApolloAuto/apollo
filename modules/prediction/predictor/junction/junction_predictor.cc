/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/predictor/junction/junction_predictor.h"

#include <utility>

#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;

void JunctionPredictor::Predict(Obstacle* obstacle) {
  // TODO(all) implement
  Clear();
  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  const Feature& latest_feature = obstacle->latest_feature();
  std::vector<JunctionExit> junction_exits =
      MostLikelyJunctions(latest_feature);
  for (const auto& junction_exit : junction_exits) {
    std::vector<TrajectoryPoint> trajectory_points;
    DrawJunctionTrajectory(latest_feature, junction_exit, &trajectory_points);
    Trajectory trajectory = GenerateTrajectory(trajectory_points);
    trajectories_.push_back(std::move(trajectory));
  }
}

void JunctionPredictor::DrawJunctionTrajectory(
    const Feature& feature,
    const JunctionExit& junction_exit,
    std::vector<TrajectoryPoint>* trajectory_points) {
  // TODO(all) implement
}

std::vector<JunctionExit> MostLikelyJunctions(const Feature& feature) {
  // TODO(all) implement
  std::vector<JunctionExit> junction_exits;
  return junction_exits;
}

}  // namespace prediction
}  // namespace apollo
