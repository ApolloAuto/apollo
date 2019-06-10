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

#include "modules/prediction/evaluator/pedestrian/pedestrian_interaction_evaluator.h"

#include <utility>
#include <vector>

#include "modules/common/math/vec2d.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::Point3D;
using apollo::common::TrajectoryPoint;
using apollo::common::adapter::AdapterConfig;
using apollo::common::math::Vec2d;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

PedestrianInteractionEvaluator::PedestrianInteractionEvaluator()
    : device_(torch::kCPU) {
  LoadModel();
}

void PedestrianInteractionEvaluator::Clear() {
  auto ptr_obstacles_container =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  std::vector<int> keys_to_delete;
  for (const auto& item : obstacle_id_lstm_state_map_) {
    int key = item.first;
    if (obstacle_id_lstm_state_map_[key].timestamp + FLAGS_max_history_time <
        ptr_obstacles_container->timestamp()) {
      keys_to_delete.push_back(key);
    }
  }
  for (const int key : keys_to_delete) {
    obstacle_id_lstm_state_map_.erase(key);
  }
}

void PedestrianInteractionEvaluator::LoadModel() {
  torch::set_num_threads(1);
  torch_position_embedding_ptr_ = torch::jit::load(
      FLAGS_torch_pedestrian_interaction_position_embedding_file, device_);
  torch_social_embedding_ptr_ = torch::jit::load(
      FLAGS_torch_pedestrian_interaction_social_embedding_file, device_);
  torch_single_lstm_ptr_ = torch::jit::load(
      FLAGS_torch_pedestrian_interaction_single_lstm_file, device_);
  torch_prediction_layer_ptr_ = torch::jit::load(
      FLAGS_torch_pedestrian_interaction_prediction_layer_file, device_);
}

torch::Tensor PedestrianInteractionEvaluator::GetSocialPooling() {
  // TODO(kechxu) implement more sophisticated logics
  return torch::zeros({1, kGridSize * kGridSize * kHiddenSize});
}

bool PedestrianInteractionEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  int id = obstacle_ptr->id();
  if (!obstacle_ptr->latest_feature().IsInitialized()) {
    AERROR << "Obstacle [" << id << "] has no latest feature.";
    return false;
  }
  Feature* latest_feature_ptr = obstacle_ptr->mutable_latest_feature();
  CHECK_NOTNULL(latest_feature_ptr);

  // Extract features, and:
  //  - if in offline mode, save it locally for training.
  //  - if in online mode, pass it through trained model to evaluate.
  std::vector<double> feature_values;
  ExtractFeatures(obstacle_ptr, &feature_values);
  if (FLAGS_prediction_offline_mode == 2) {
    FeatureOutput::InsertDataForLearning(*latest_feature_ptr, feature_values,
                                         "pedestrian", nullptr);
    ADEBUG << "Saving extracted features for learning locally.";
    return true;
  }
  // Step 1 Get social embedding
  torch::Tensor social_pooling = GetSocialPooling();
  std::vector<torch::jit::IValue> social_embedding_inputs;
  social_embedding_inputs.push_back(std::move(social_pooling));
  torch::Tensor social_embedding =
      torch_social_embedding_ptr_->forward(social_embedding_inputs)
          .toTensor()
          .to(torch::kCPU);

  // Step 2 Get position embedding
  double pos_x = feature_values[2];
  double pos_y = feature_values[3];
  torch::Tensor torch_position = torch::zeros({1, 2});
  torch_position[0][0] = pos_x;
  torch_position[0][1] = pos_y;
  std::vector<torch::jit::IValue> position_embedding_inputs;
  position_embedding_inputs.push_back(std::move(torch_position));
  torch::Tensor position_embedding =
      torch_position_embedding_ptr_->forward(position_embedding_inputs)
          .toTensor()
          .to(torch::kCPU);

  // Step 3 Conduct single LSTM and update hidden states
  torch::Tensor lstm_input =
      torch::zeros({1, 2 * (kEmbeddingSize + kHiddenSize)});
  for (int i = 0; i < kEmbeddingSize; ++i) {
    lstm_input[0][i] = position_embedding[0][i];
    lstm_input[0][kEmbeddingSize + i] = social_embedding[0][i];
  }
  torch::Tensor curr_ht = torch::zeros({1, kHiddenSize});
  torch::Tensor curr_ct = torch::zeros({1, kHiddenSize});
  if (obstacle_id_lstm_state_map_.find(id) !=
      obstacle_id_lstm_state_map_.end()) {
    curr_ht = obstacle_id_lstm_state_map_[id].ht;
    curr_ct = obstacle_id_lstm_state_map_[id].ct;
  }
  for (int i = 0; i < kHiddenSize; ++i) {
    lstm_input[0][2 * kEmbeddingSize + i] = curr_ht[0][i];
    lstm_input[0][2 * kEmbeddingSize + kHiddenSize + i] = curr_ct[0][i];
  }
  std::vector<torch::jit::IValue> lstm_inputs;
  lstm_inputs.push_back(std::move(lstm_input));
  auto lstm_out_tuple = torch_single_lstm_ptr_->forward(lstm_inputs).toTuple();
  auto new_ht = lstm_out_tuple->elements()[0].toTensor();
  auto new_ct = lstm_out_tuple->elements()[1].toTensor();
  obstacle_id_lstm_state_map_[id].ht = new_ht;
  obstacle_id_lstm_state_map_[id].ct = new_ct;

  // Step 4 for-loop get a trajectory
  // Set the starting trajectory point
  Trajectory* trajectory = latest_feature_ptr->add_predicted_trajectory();
  trajectory->set_probability(1.0);
  TrajectoryPoint* start_point = trajectory->add_trajectory_point();
  start_point->mutable_path_point()->set_x(pos_x);
  start_point->mutable_path_point()->set_y(pos_y);
  start_point->mutable_path_point()->set_theta(latest_feature_ptr->theta());
  start_point->set_v(latest_feature_ptr->speed());
  start_point->set_relative_time(0.0);

  int num_trajectory_point =
      static_cast<int>(FLAGS_prediction_trajectory_time_length /
                       FLAGS_prediction_trajectory_time_resolution);
  for (int i = 1; i < num_trajectory_point; ++i) {
    double prev_x = trajectory->trajectory_point(i - 1).path_point().x();
    double prev_y = trajectory->trajectory_point(i - 1).path_point().y();
    Point3D position =
        PredictNextPosition(id, prev_x, prev_y, social_embedding);
    TrajectoryPoint* point = trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(position.x());
    point->mutable_path_point()->set_y(position.y());
    Vec2d direction(position.x() - prev_x, position.y() - prev_y);
    double distance = direction.Length();
    point->set_v(distance / FLAGS_prediction_trajectory_time_resolution);
    point->mutable_path_point()->set_theta(direction.Angle());
    point->set_relative_time(
        static_cast<double>(FLAGS_prediction_trajectory_time_resolution * i));
  }

  return true;
}

Point3D PedestrianInteractionEvaluator::PredictNextPosition(
    const int obstacle_id, const double pos_x, const double pos_y,
    const torch::Tensor& social_embedding) {
  Point3D point;
  CHECK(obstacle_id_lstm_state_map_.find(obstacle_id) !=
        obstacle_id_lstm_state_map_.end());
  const auto& ht = obstacle_id_lstm_state_map_[obstacle_id].ht;
  const auto& ct = obstacle_id_lstm_state_map_[obstacle_id].ct;
  torch::Tensor torch_position = torch::zeros({1, 2});
  torch_position[0][0] = pos_x;
  torch_position[0][1] = pos_y;
  std::vector<torch::jit::IValue> position_embedding_inputs;
  position_embedding_inputs.push_back(std::move(torch_position));
  torch::Tensor position_embedding =
      torch_position_embedding_ptr_->forward(position_embedding_inputs)
          .toTensor()
          .to(torch::kCPU);
  torch::Tensor lstm_input =
      torch::zeros({1, 2 * (kEmbeddingSize + kHiddenSize)});
  for (int i = 0; i < kEmbeddingSize; ++i) {
    lstm_input[0][i] = position_embedding[0][i];
    lstm_input[0][kEmbeddingSize + i] = social_embedding[0][i];
  }
  for (int i = 0; i < kHiddenSize; ++i) {
    lstm_input[0][2 * kEmbeddingSize + i] = ht[0][0][i];
    lstm_input[0][2 * kEmbeddingSize + kHiddenSize + i] = ct[0][0][i];
  }
  std::vector<torch::jit::IValue> lstm_inputs;
  lstm_inputs.push_back(std::move(lstm_input));
  auto lstm_out_tuple = torch_single_lstm_ptr_->forward(lstm_inputs).toTuple();
  auto new_ht = lstm_out_tuple->elements()[0].toTensor()[0];
  std::vector<torch::jit::IValue> prediction_inputs;
  prediction_inputs.push_back(std::move(new_ht));
  auto pred_out_tensor = torch_prediction_layer_ptr_->forward(prediction_inputs)
                             .toTensor()
                             .to(torch::kCPU);
  auto pred_out = pred_out_tensor.accessor<float, 2>();
  point.set_x(static_cast<double>(pred_out[0][0]));
  point.set_y(static_cast<double>(pred_out[0][1]));
  return point;
}

bool PedestrianInteractionEvaluator::ExtractFeatures(
    const Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);
  CHECK_NOTNULL(feature_values);

  // Extract obstacle related features.
  double timestamp = obstacle_ptr->latest_feature().timestamp();
  int id = obstacle_ptr->latest_feature().id();
  double pos_x = obstacle_ptr->latest_feature().position().x();
  double pos_y = obstacle_ptr->latest_feature().position().y();

  // Insert it into the feature_values.
  feature_values->push_back(timestamp);
  feature_values->push_back(id * 1.0);
  feature_values->push_back(pos_x);
  feature_values->push_back(pos_y);

  return true;
}

}  // namespace prediction
}  // namespace apollo
