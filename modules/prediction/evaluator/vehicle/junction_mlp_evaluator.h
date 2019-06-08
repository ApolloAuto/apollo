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

class JunctionMLPEvaluator : public Evaluator {
 public:
  /**
   * @brief Constructor
   */
  JunctionMLPEvaluator();

  /**
   * @brief Destructor
   */
  virtual ~JunctionMLPEvaluator() = default;

  /**
   * @brief Clear obstacle feature map
   */
  void Clear();

  /**
   * @brief Override Evaluate
   * @param Obstacle pointer
   */
  bool Evaluate(Obstacle* obstacle_ptr) override;

  /**
   * @brief Extract feature vector
   * @param Obstacle pointer
   *        Feature container in a vector for receiving the feature values
   */
  void ExtractFeatureValues(Obstacle* obstacle_ptr,
                            std::vector<double>* feature_values);

  /**
   * @brief Get the name of evaluator.
   */
  std::string GetName() override { return "JUNCTION_MLP_EVALUATOR"; }

 private:
  /**
   * @brief Set obstacle feature vector
   * @param Obstacle pointer
   *        Feature container in a vector for receiving the feature values
   */
  void SetObstacleFeatureValues(Obstacle* obstacle_ptr,
                                std::vector<double>* const feature_values);

  /**
   * @brief Set ego vehicle feature vector
   * @param Obstacle pointer
   *        Feature container in a vector for receiving the feature values
   */
  void SetEgoVehicleFeatureValues(Obstacle* obstacle_ptr,
                                  std::vector<double>* const feature_values);

  /**
   * @brief Set junction feature vector
   * @param Obstacle pointer
   *        Feature container in a vector for receiving the feature values
   */
  void SetJunctionFeatureValues(Obstacle* obstacle_ptr,
                                std::vector<double>* const feature_values);

  /**
   * @brief Load model file
   */
  void LoadModel();

 private:
  // obstacle feature with 4 basic features and 5 frames of history position
  static const size_t OBSTACLE_FEATURE_SIZE = 4 + 2 * 5;
  // ego vehicle feature of position and velocity
  static const size_t EGO_VEHICLE_FEATURE_SIZE = 4;
  // junction feature on 12 fan area 8 dim each
  static const size_t JUNCTION_FEATURE_SIZE = 12 * 8;

  std::shared_ptr<torch::jit::script::Module> torch_model_ptr_ = nullptr;
  torch::Device device_;
};

}  // namespace prediction
}  // namespace apollo
