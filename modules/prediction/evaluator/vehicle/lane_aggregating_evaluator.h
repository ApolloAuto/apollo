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

/**
 * @file
 * @brief Define the lane aggregating evaluator class
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "torch/script.h"
#include "torch/torch.h"

#include "modules/prediction/evaluator/evaluator.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

class LaneAggregatingEvaluator : public Evaluator {
 public:
  /**
   * @brief Constructor
   */
  LaneAggregatingEvaluator();

  /**
   * @brief Destructor
   */
  virtual ~LaneAggregatingEvaluator() = default;

  /**
   * @brief Override Evaluate
   * @param Obstacle pointer
   */
  bool Evaluate(Obstacle* obstacle_ptr) override;

  /**
   * @brief Get the name of evaluator.
   */
  std::string GetName() override { return "LANE_AGGREGATING_EVALUATOR"; }

 private:
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
  bool ExtractStaticEnvFeatures(
      const Obstacle* obstacle_ptr, const LaneGraph* lane_graph_ptr,
      std::vector<std::vector<double>>* feature_values,
      std::vector<int>* lane_sequence_idx_to_remove);

  torch::Tensor AggregateLaneEncodings(
      const std::vector<torch::Tensor>& lane_encoding_list);

  torch::Tensor LaneEncodingMaxPooling(
      const std::vector<torch::Tensor>& lane_encoding_list);

  torch::Tensor LaneEncodingAvgPooling(
      const std::vector<torch::Tensor>& lane_encoding_list);

  std::vector<double> StableSoftmax(
      const std::vector<double>& prediction_scores);

  void LoadModel();

 private:
  std::shared_ptr<torch::jit::script::Module> torch_obstacle_encoding_ptr_ =
      nullptr;
  std::shared_ptr<torch::jit::script::Module> torch_lane_encoding_ptr_ =
      nullptr;
  std::shared_ptr<torch::jit::script::Module> torch_prediction_layer_ptr_ =
      nullptr;
  torch::Device device_;

  static const size_t OBSTACLE_FEATURE_SIZE = 20 * 9;
  static const size_t SINGLE_LANE_FEATURE_SIZE = 4;
  static const size_t LANE_POINTS_SIZE = 100;          // 50m
  static const size_t BACKWARD_LANE_POINTS_SIZE = 50;  // 25m

  static const size_t OBSTACLE_ENCODING_SIZE = 128;
  static const size_t SINGLE_LANE_ENCODING_SIZE = 128;
  static const size_t AGGREGATED_ENCODING_SIZE = 256;
};

}  // namespace prediction
}  // namespace apollo
