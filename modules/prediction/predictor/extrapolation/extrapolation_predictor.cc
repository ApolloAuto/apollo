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

void ExtrapolationPredictor::Predict(
    const ADCTrajectoryContainer* adc_trajectory_container, Obstacle* obstacle,
    ObstaclesContainer* obstacles_container) {
  Clear();

  CHECK_NOTNULL(obstacle);
  CHECK_GT(obstacle->history_size(), 0);

  obstacle->SetPredictorType(predictor_type_);

  Feature* feature_ptr = obstacle->mutable_latest_feature();

  if (!feature_ptr->lane().has_lane_graph()) {
    AERROR << "Obstacle [" << obstacle->id() << "] has no lane graph.";
    return;
  }
  for (int i = 0; i < feature_ptr->predicted_trajectory_size(); ++i) {
    Trajectory* trajectory_ptr = feature_ptr->mutable_predicted_trajectory(i);
    PostProcess(trajectory_ptr);
  }
}

void ExtrapolationPredictor::PostProcess(Trajectory* trajectory_ptr) {
  // TODO(kechxu) handle corner cases
  auto lane_search_result = SearchExtrapolationLane(*trajectory_ptr);
  if (lane_search_result.found) {
    ExtrapolateByLane(lane_search_result.lane_id, trajectory_ptr);
  } else {
    ExtrapolateByFreeMove(trajectory_ptr);
  }
}

ExtrapolationPredictor::LaneSearchResult
ExtrapolationPredictor::SearchExtrapolationLane(const Trajectory& trajectory) {
  LaneSearchResult lane_search_result;
  // TODO(kechxu) implement
  return lane_search_result;
}

void ExtrapolationPredictor::ExtrapolateByLane(
    const std::string& lane_id, Trajectory* trajectory_ptr) {
  // TODO(kechxu) implement
}

void ExtrapolationPredictor::ExtrapolateByFreeMove(Trajectory* trajectory_ptr) {
  // TODO(kechxu) implement
}

}  // namespace prediction
}  // namespace apollo
