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

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/prediction/common/semantic_map.h"
#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/evaluator/model_manager/model_manager.h"
#include "torch/extension.h"
#include "torch/script.h"

namespace apollo {
namespace prediction {

class SemanticLSTMEvaluator : public Evaluator {
 public:
  /**
   * @brief Constructor
   */
  SemanticLSTMEvaluator() = delete;
  explicit SemanticLSTMEvaluator(SemanticMap* semantic_map);

  /**
   * @brief Destructor
   */
  virtual ~SemanticLSTMEvaluator() = default;

  /**
   * @brief Clear obstacle feature map
   */
  void Clear();

  /**
   * @brief Override Evaluate
   * @param Obstacle pointer
   * @param Obstacles container
   */
  bool Evaluate(Obstacle* obstacle_ptr,
                ObstaclesContainer* obstacles_container) override;

  /**
   * @brief Extract obstacle history
   * @param Obstacle pointer
   *        Feature container in a vector for receiving the obstacle history
   */
  bool ExtractObstacleHistory(
      Obstacle* obstacle_ptr,
      std::vector<std::pair<double, double>>* pos_history);

  /**
   * @brief Get the name of evaluator.
   */
  std::string GetName() override { return "SEMANTIC_LSTM_EVALUATOR"; }

 private:
  /**
   * @brief Load model file
   */
  void LoadModel();

 private:
  ModelManager model_manager_;
  at::Tensor torch_default_output_tensor_;
  Model::Backend device_;
  SemanticMap* semantic_map_;
};

}  // namespace prediction
}  // namespace apollo
