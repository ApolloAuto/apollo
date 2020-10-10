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

#include <string>
#include <vector>

#include "torch/script.h"
#include "torch/torch.h"

#include "modules/prediction/evaluator/evaluator.h"

#include "modules/prediction/container/obstacles/obstacles_container.h"

namespace apollo {
namespace prediction {

class CruiseMLPEvaluator : public Evaluator {
 public:
  /**
   * @brief Constructor
   */
  CruiseMLPEvaluator();

  /**
   * @brief Destructor
   */
  virtual ~CruiseMLPEvaluator() = default;

  /**
   * @brief Override Evaluate
   * @param Obstacle pointer
   * @param Obstacles container
   */
  bool Evaluate(Obstacle* obstacle_ptr,
                ObstaclesContainer* obstacles_container) override;

  /**
   * @brief Extract feature vector
   * @param Obstacle pointer
   *        Lane Sequence pointer
   */
  void ExtractFeatureValues(Obstacle* obstacle_ptr,
                            LaneSequence* lane_sequence_ptr,
                            std::vector<double>* feature_values);

  /**
   * @brief Get the name of evaluator.
   */
  std::string GetName() override { return "CRUISE_MLP_EVALUATOR"; }

  void Clear();

 private:
  /**
   * @brief Set obstacle feature vector
   * @param Obstacle pointer
   *        Feature container in a vector for receiving the feature values
   */
  void SetObstacleFeatureValues(const Obstacle* obstacle_ptr,
                                std::vector<double>* feature_values);

  /**
   * @brief Set interaction feature vector
   * @param Obstacle pointer
   * @param Obstacles container
   * @param Lane sequence pointer
   * @param Feature container in a vector for receiving the feature values
   */
  void SetInteractionFeatureValues(Obstacle* obstacle_ptr,
                                   ObstaclesContainer* obstacles_container,
                                   LaneSequence* lane_sequence_ptr,
                                   std::vector<double>* feature_values);

  /**
   * @brief Set lane feature vector
   * @param Obstacle pointer
   *        Lane sequence pointer
   *        Feature container in a vector for receiving the feature values
   */
  void SetLaneFeatureValues(const Obstacle* obstacle_ptr,
                            const LaneSequence* lane_sequence_ptr,
                            std::vector<double>* feature_values);

  /**
   * @brief Load model files
   */
  void LoadModels();

  void ModelInference(const std::vector<torch::jit::IValue>& torch_inputs,
                      torch::jit::script::Module torch_model_ptr,
                      LaneSequence* lane_sequence_ptr);

 private:
  static const size_t OBSTACLE_FEATURE_SIZE = 23 + 5 * 9;
  static const size_t INTERACTION_FEATURE_SIZE = 8;
  static const size_t SINGLE_LANE_FEATURE_SIZE = 4;
  static const size_t LANE_POINTS_SIZE = 20;

  torch::jit::script::Module torch_go_model_;
  torch::jit::script::Module torch_cutin_model_;
  torch::Device device_;
};

}  // namespace prediction
}  // namespace apollo
