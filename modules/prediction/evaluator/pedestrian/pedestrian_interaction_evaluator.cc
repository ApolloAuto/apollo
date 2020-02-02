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

#include "modules/common/math/vec2d.h"
#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_constants.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::TrajectoryPoint;

PedestrianInteractionEvaluator::PedestrianInteractionEvaluator()
    : device_(torch::kCPU) {
  evaluator_type_ = ObstacleConf::PEDESTRIAN_INTERACTION_EVALUATOR;
  LoadModel();
}

/* TODO(kechxu) figure out if this function is necessary. It is not being used
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
*/

void PedestrianInteractionEvaluator::LoadModel() {
  torch::set_num_threads(1);
  if (FLAGS_use_cuda && torch::cuda::is_available()) {
    ADEBUG << "CUDA is available";
    device_ = torch::Device(torch::kCUDA);
  }

  torch_position_embedding_ = torch::jit::load(
      FLAGS_torch_pedestrian_interaction_position_embedding_file, device_);
  torch_social_embedding_ = torch::jit::load(
      FLAGS_torch_pedestrian_interaction_social_embedding_file, device_);
  torch_single_lstm_ = torch::jit::load(
      FLAGS_torch_pedestrian_interaction_single_lstm_file, device_);
  torch_prediction_layer_ = torch::jit::load(
      FLAGS_torch_pedestrian_interaction_prediction_layer_file, device_);
}

torch::Tensor PedestrianInteractionEvaluator::GetSocialPooling() {
  // TODO(kechxu) implement more sophisticated logics
  return torch::zeros({1, kGridSize * kGridSize * kHiddenSize});
}

bool PedestrianInteractionEvaluator::Evaluate(
    Obstacle* obstacle_ptr, ObstaclesContainer* obstacles_container) {
  // Sanity checks.
  CHECK_NOTNULL(obstacle_ptr);

  obstacle_ptr->SetEvaluatorType(evaluator_type_);

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
  if (FLAGS_prediction_offline_mode ==
      PredictionConstants::kDumpDataForLearning) {
    FeatureOutput::InsertDataForLearning(*latest_feature_ptr, feature_values,
                                         "pedestrian", nullptr);
    ADEBUG << "Saving extracted features for learning locally.";
    return true;
  }

  static constexpr double kShortTermPredictionTimeResolution = 0.4;
  static constexpr int kShortTermPredictionPointNum = 5;
  static constexpr int kHiddenStateUpdateCycle = 4;

  // Step 1 Get social embedding
  torch::Tensor social_pooling = GetSocialPooling();
  std::vector<torch::jit::IValue> social_embedding_inputs;
  social_embedding_inputs.push_back(std::move(social_pooling.to(device_)));
  torch::Tensor social_embedding =
      torch_social_embedding_.forward(social_embedding_inputs)
          .toTensor()
          .to(torch::kCPU);

  // Step 2 Get position embedding
  double pos_x = feature_values[2];
  double pos_y = feature_values[3];
  double rel_x = 0.0;
  double rel_y = 0.0;
  if (obstacle_ptr->history_size() > kHiddenStateUpdateCycle - 1) {
    rel_x = obstacle_ptr->latest_feature().position().x() -
            obstacle_ptr->feature(3).position().x();
    rel_y = obstacle_ptr->latest_feature().position().y() -
            obstacle_ptr->feature(3).position().y();
  }

  torch::Tensor torch_position = torch::zeros({1, 2});
  torch_position[0][0] = rel_x;
  torch_position[0][1] = rel_y;
  std::vector<torch::jit::IValue> position_embedding_inputs;
  position_embedding_inputs.push_back(std::move(torch_position.to(device_)));
  torch::Tensor position_embedding =
      torch_position_embedding_.forward(position_embedding_inputs)
          .toTensor()
          .to(torch::kCPU);

  // Step 3 Conduct single LSTM and update hidden states
  torch::Tensor lstm_input =
      torch::zeros({1, 2 * (kEmbeddingSize + kHiddenSize)});
  for (int i = 0; i < kEmbeddingSize; ++i) {
    lstm_input[0][i] = position_embedding[0][i];
  }

  if (obstacle_id_lstm_state_map_.find(id) ==
      obstacle_id_lstm_state_map_.end()) {
    obstacle_id_lstm_state_map_[id].ht = torch::zeros({1, 1, kHiddenSize});
    obstacle_id_lstm_state_map_[id].ct = torch::zeros({1, 1, kHiddenSize});
    obstacle_id_lstm_state_map_[id].timestamp = obstacle_ptr->timestamp();
    obstacle_id_lstm_state_map_[id].frame_count = 0;
  }
  torch::Tensor curr_ht = obstacle_id_lstm_state_map_[id].ht;
  torch::Tensor curr_ct = obstacle_id_lstm_state_map_[id].ct;
  int curr_frame_count = obstacle_id_lstm_state_map_[id].frame_count;

  if (curr_frame_count == kHiddenStateUpdateCycle - 1) {
    for (int i = 0; i < kHiddenSize; ++i) {
      lstm_input[0][kEmbeddingSize + i] = curr_ht[0][0][i];
      lstm_input[0][kEmbeddingSize + kHiddenSize + i] = curr_ct[0][0][i];
    }

    std::vector<torch::jit::IValue> lstm_inputs;
    lstm_inputs.push_back(std::move(lstm_input.to(device_)));
    auto lstm_out_tuple = torch_single_lstm_.forward(lstm_inputs).toTuple();
    auto ht = lstm_out_tuple->elements()[0].toTensor();
    auto ct = lstm_out_tuple->elements()[1].toTensor();
    obstacle_id_lstm_state_map_[id].ht = ht.clone();
    obstacle_id_lstm_state_map_[id].ct = ct.clone();
  }
  obstacle_id_lstm_state_map_[id].frame_count =
      (curr_frame_count + 1) % kHiddenStateUpdateCycle;

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

  for (int i = 1; i <= kShortTermPredictionPointNum; ++i) {
    double prev_x = trajectory->trajectory_point(i - 1).path_point().x();
    double prev_y = trajectory->trajectory_point(i - 1).path_point().y();
    CHECK(obstacle_id_lstm_state_map_.find(id) !=
          obstacle_id_lstm_state_map_.end());
    torch::Tensor torch_position = torch::zeros({1, 2});
    double curr_rel_x = rel_x;
    double curr_rel_y = rel_y;
    if (i > 1) {
      curr_rel_x =
          prev_x - trajectory->trajectory_point(i - 2).path_point().x();
      curr_rel_y =
          prev_y - trajectory->trajectory_point(i - 2).path_point().y();
    }
    torch_position[0][0] = curr_rel_x;
    torch_position[0][1] = curr_rel_y;
    std::vector<torch::jit::IValue> position_embedding_inputs;
    position_embedding_inputs.push_back(std::move(torch_position.to(device_)));
    torch::Tensor position_embedding =
        torch_position_embedding_.forward(position_embedding_inputs)
            .toTensor()
            .to(torch::kCPU);
    torch::Tensor lstm_input =
        torch::zeros({1, kEmbeddingSize + 2 * kHiddenSize});
    for (int i = 0; i < kEmbeddingSize; ++i) {
      lstm_input[0][i] = position_embedding[0][i];
    }

    auto ht = obstacle_id_lstm_state_map_[id].ht.clone();
    auto ct = obstacle_id_lstm_state_map_[id].ct.clone();

    for (int i = 0; i < kHiddenSize; ++i) {
      lstm_input[0][kEmbeddingSize + i] = ht[0][0][i];
      lstm_input[0][kEmbeddingSize + kHiddenSize + i] = ct[0][0][i];
    }
    std::vector<torch::jit::IValue> lstm_inputs;
    lstm_inputs.push_back(std::move(lstm_input.to(device_)));
    auto lstm_out_tuple = torch_single_lstm_.forward(lstm_inputs).toTuple();
    ht = lstm_out_tuple->elements()[0].toTensor();
    ct = lstm_out_tuple->elements()[1].toTensor();
    std::vector<torch::jit::IValue> prediction_inputs;
    prediction_inputs.push_back(ht[0]);
    auto pred_out_tensor = torch_prediction_layer_.forward(prediction_inputs)
                               .toTensor()
                               .to(torch::kCPU);
    auto pred_out = pred_out_tensor.accessor<float, 2>();
    TrajectoryPoint* point = trajectory->add_trajectory_point();
    double curr_x = prev_x + static_cast<double>(pred_out[0][0]);
    double curr_y = prev_y + static_cast<double>(pred_out[0][1]);
    point->mutable_path_point()->set_x(curr_x);
    point->mutable_path_point()->set_y(curr_y);
    point->set_v(latest_feature_ptr->speed());
    point->mutable_path_point()->set_theta(
        latest_feature_ptr->velocity_heading());
    point->set_relative_time(kShortTermPredictionTimeResolution *
                             static_cast<double>(i));
  }

  return true;
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
