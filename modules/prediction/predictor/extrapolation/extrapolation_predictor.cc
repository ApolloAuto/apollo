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

#include "modules/prediction/predictor/extrapolation/extrapolation_predictor.h"

namespace apollo {
namespace prediction {

ExtrapolationPredictor::ExtrapolationPredictor() {
  predictor_type_ = ObstacleConf::EXTRAPOLATION_PREDICTOR;
}

void ExtrapolationPredictor::Predict(Obstacle* obstacle) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  obstacle->SetPredictorType(predictor_type_);

  const Feature& feature = obstacle->latest_feature();

  if (!feature.has_lane() || !feature.lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << "] has no lane graph.";
    return;
  }
  std::vector<apollo::common::TrajectoryPoint> trajectory_points;
  DrawShortTermTrajectory(feature, &trajectory_points);
  Trajectory trajectory = GenerateTrajectory(trajectory_points);
  obstacle->mutable_latest_feature()->add_predicted_trajectory()->CopyFrom(
      trajectory);
}

void ExtrapolationPredictor::DrawShortTermTrajectory(
    const Feature& feature,
    std::vector<apollo::common::TrajectoryPoint>* points) {
  for (const auto& point : feature.short_term_predicted_trajectory_points()) {
    points->push_back(point);
  }
}

}  // namespace prediction
}  // namespace apollo
