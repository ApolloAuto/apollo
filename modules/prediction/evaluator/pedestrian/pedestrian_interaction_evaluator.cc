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

#include <vector>

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/container/container_manager.h"
#include "modules/prediction/container/obstacles/obstacles_container.h"
#include "modules/prediction/container/pose/pose_container.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

PedestrianInteractionEvaluator::PedestrianInteractionEvaluator()
    : device_(torch::kCPU) { }

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

torch::jit::IValue PedestrianInteractionEvaluator::GetSocialPooling() {
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
  // TODO(all) implement the online part.
  // Step 1 Get social embedding

  // Step 2 Get position embedding

  // Step 3 Conduct single LSTM and update hidden states

  // Step 4 for-loop get a trajectory

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
  auto ptr_ego_pose_container =
      ContainerManager::Instance()->GetContainer<PoseContainer>(
          AdapterConfig::LOCALIZATION);
  CHECK_NOTNULL(ptr_ego_pose_container);
  const PerceptionObstacle* ptr_ego_vehicle =
      ptr_ego_pose_container->ToPerceptionObstacle();
  double adc_x = ptr_ego_vehicle->position().x();
  double adc_y = ptr_ego_vehicle->position().y();

  // Insert it into the feature_values.
  feature_values->push_back(timestamp);
  feature_values->push_back(id * 1.0);
  feature_values->push_back(pos_x);
  feature_values->push_back(pos_y);
  feature_values->push_back(adc_x);
  feature_values->push_back(adc_y);

  return true;
}

}  // namespace prediction
}  // namespace apollo
