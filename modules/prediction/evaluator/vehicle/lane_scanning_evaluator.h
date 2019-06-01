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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "torch/script.h"
#include "torch/torch.h"

#include "modules/prediction/evaluator/evaluator.h"

namespace apollo {
namespace prediction {

class LaneScanningEvaluator : public Evaluator {
 public:
  /**
   * @brief Constructor
   */
  LaneScanningEvaluator();

  /**
   * @brief Destructor
   */
  virtual ~LaneScanningEvaluator() = default;

  /**
   * @brief Override Evaluate
   * @param Obstacle pointer
   */
  bool Evaluate(Obstacle* obstacle_ptr) override;

  /**
   * @brief Override Evaluate
   * @param Obstacle pointer
   * @param vector of all Obstacles
   */
  bool Evaluate(Obstacle* obstacle_ptr,
                std::vector<Obstacle*> dynamic_env) override;

  /**
   * @brief Extract features for learning model's input
   * @param Obstacle pointer
   * @param Lane Graph pointer
   * @param To be filled up with extracted features
   */
  bool ExtractFeatures(const Obstacle* obstacle_ptr,
                       const LaneGraph* lane_graph_ptr,
                       std::vector<double>* feature_values);

  bool ExtractStringFeatures(
      const LaneGraph& lane_graph,
      std::vector<std::string>* const string_feature_values);

  /**
   * @brief Get the name of evaluator.
   */
  std::string GetName() override { return "LANE_SCANNING_EVALUATOR"; }

 private:
  /**
   * @brief Load model from file
   */
  void LoadModel();

  /**
   * @brief Extract the features for obstacles
   * @param Obstacle pointer
   *        A vector of doubles to be filled up with extracted features
   */
  bool ExtractObstacleFeatures(const Obstacle* obstacle_ptr,
                               std::vector<double>* feature_values);

  /**
   * @brief Set lane feature vector
   * @param Obstacle pointer
   *        A vector of doubles to be filled up with extracted features
   */
  bool ExtractStaticEnvFeatures(const Obstacle* obstacle_ptr,
                                const LaneGraph* lane_graph_ptr,
                                std::vector<double>* feature_values,
                                std::vector<int>* lane_sequence_idx_to_remove);

  void ModelInference(
      const std::vector<torch::jit::IValue>& torch_inputs,
      std::shared_ptr<torch::jit::script::Module> torch_model_ptr,
      Feature* feature_ptr);

 private:
  static const size_t OBSTACLE_FEATURE_SIZE = 20 * 9;
  static const size_t INTERACTION_FEATURE_SIZE = 8;
  static const size_t SINGLE_LANE_FEATURE_SIZE = 4;
  static const size_t LANE_POINTS_SIZE = 100;          // 50m
  static const size_t BACKWARD_LANE_POINTS_SIZE = 50;  // 25m
  static const size_t MAX_NUM_LANE = 10;
  static const size_t SHORT_TERM_TRAJECTORY_SIZE = 10;

  std::shared_ptr<torch::jit::script::Module> torch_lane_scanning_model_ptr_ =
      nullptr;
  torch::Device device_;
};

}  // namespace prediction
}  // namespace apollo
