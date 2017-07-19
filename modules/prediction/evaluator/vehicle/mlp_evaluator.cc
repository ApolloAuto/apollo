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

#include "modules/prediction/evaluator/vehicle/mlp_evaluator.h"

namespace apollo {
namespace prediction {

MLPEvaluator::MLPEvaluator() {
}

MLPEvaluator::~MLPEvaluator() {
}

void MLPEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  Clear();
  if (obstacle_ptr == nullptr) {
    AERROR << "Invalid obstacle.";
    return;
  }

  int id = obstacle_ptr->id();
  Feature latest_feature = obstacle_ptr->latest_feature();
  if (!latest_feature.IsInitialized()) {
    ADEBUG << "Obstacle [" << id << "] has no latest feature.";
    return;
  }

  Lane* lane_ptr = latest_feature.mutable_lane();
  if (!latest_feature.has_lane() || lane_ptr == nullptr) {
    ADEBUG << "Obstacle [" << id << "] has no lane feature.";
    return;
  }

  LaneGraph* lane_graph_ptr = lane_ptr->mutable_lane_graph();
  if (!latest_feature.lane().has_lane_graph() || lane_graph_ptr == nullptr) {
    ADEBUG << "Obstacle [" << id << "] has no lane graph.";
    return;
  }

  if (latest_feature.lane().lane_graph().lane_sequence_size() == 0) {
    ADEBUG << "Obstacle [" << id << "] has no lane sequences.";
    return;
  }

  for (int i = 0;
      i < latest_feature.lane().lane_graph().lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence_ptr = lane_graph_ptr->mutable_lane_sequence(i);
    CHECK(lane_sequence_ptr != nullptr);
    ExtractFeatureValues(obstacle_ptr, lane_sequence_ptr);
  }
}

void MLPEvaluator::ExtractFeatureValues(Obstacle* obstacle_ptr,
                                        LaneSequence* lane_sequence_ptr) {
  feature_values_.clear();
  int id = obstacle_ptr->id();
  std::vector<double> obstacle_feature_values;
  if (obstacle_feature_values_map_.find(id) ==
      obstacle_feature_values_map_.end()) {
    SetObstacleFeatureValues(obstacle_ptr, &obstacle_feature_values);
  } else {
    obstacle_feature_values = obstacle_feature_values_map_[id];
  }

  if (obstacle_feature_values.size() != OBSTACLE_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected obstacle features "
           << obstacle_feature_values.size() << ".";
    return;
  }

  std::vector<double> lane_feature_values;
  SetLaneFeatureValues(obstacle_ptr, lane_sequence_ptr, &lane_feature_values);
  if (lane_feature_values.size() != LANE_FEATURE_SIZE) {
    ADEBUG << "Obstacle [" << id << "] has fewer than "
           << "expected lane features"
           << lane_feature_values.size() << ".";
    return;
  }

  feature_values_.insert(feature_values_.end(),
      lane_feature_values.begin(), lane_feature_values.end());
  feature_values_.insert(feature_values_.end(),
      lane_feature_values.begin(), lane_feature_values.end());
}

void MLPEvaluator::SetObstacleFeatureValues(Obstacle* obstacle,
    std::vector<double>* feature_values) {
  // TODO(kechxu) implement
}

void MLPEvaluator::SetLaneFeatureValues(Obstacle* obstacle,
    LaneSequence* lane_sequence, std::vector<double>* features) {
  // TODO(kechxu) implement
}

}  // namespace prediction
}  // namespace apollo
