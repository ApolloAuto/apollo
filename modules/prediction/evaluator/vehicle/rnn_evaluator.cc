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

#include "modules/prediction/evaluator/vehicle/rnn_evaluator.h"

#include "modules/common/util/file.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

RNNEvaluator::RNNEvaluator() { LoadModel(FLAGS_vehicle_model_file); }

void RNNEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  Clear();
  CHECK_NOTNULL(obstacle_ptr);

  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return;
  }

  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);
  if (!latest_feature_ptr->has_lane() ||
      !latest_feature_ptr->lane().has_lane_graph()) {
    ADEBUG << "Obstacle [" << id << "] has no lane graph.";
    return;
  }

  LaneGraph* lane_graph_ptr =
      latest_feature_ptr->mutable_lane()->mutable_lane_graph();
  CHECK_NOTNULL(lane_graph_ptr);
  if (lane_graph_ptr->lane_sequence_size() == 0) {
    AERROR << "Obstacle [" << id << "] has no lane sequences.";
    return;
  }

  // TODO(all) continue implement
}

void RNNEvaluator::Clear() {}

void RNNEvaluator::LoadModel(const std::string& model_file) {
  model_ptr_.reset(new NetParameter());
  CHECK(model_ptr_ != nullptr);
  CHECK(common::util::GetProtoFromFile(model_file, model_ptr_.get()))
      << "Unable to load model file: " << model_file << ".";

  AINFO << "Succeeded in loading the model file: " << model_file << ".";
}

void RNNEvaluator::ExtractFeatureValues(
      Obstacle* obstacle,
      Eigen::MatrixXf* const obstacle_feature_mat,
      std::unordered_map<int, Eigen::MatrixXf>* const lane_feature_mats) {
  // TODO(all) implement
}

}  // namespace prediction
}  // namespace apollo
