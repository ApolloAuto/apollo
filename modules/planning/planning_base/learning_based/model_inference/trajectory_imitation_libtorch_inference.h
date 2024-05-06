/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @brief Define the trajectory_imitation_libtorch_inference class
 */

#pragma once

#include <string>

#include "torch/extension.h"
#include "torch/script.h"

#include "modules/planning/planning_base/learning_based/model_inference/model_inference.h"

namespace apollo {
namespace planning {

class TrajectoryImitationLibtorchInference : public ModelInference {
 public:
  /**
   * @brief Constructor
   */
  explicit TrajectoryImitationLibtorchInference(
      const LearningModelInferenceTaskConfig& config);

  /**
   * @brief Destructor
   */
  virtual ~TrajectoryImitationLibtorchInference() = default;

  /**
   * @brief Get the name of model inference
   */
  std::string GetName() override { return "TRAJECTORY_IMITATION_INFERENCE"; };

  /**
   * @brief load a learned model
   */
  bool LoadModel() override;

  /**
   * @brief inference a learned model
   * @param learning_data_frame input and output intermediate for inference
   */
  bool DoInference(LearningDataFrame* const learning_data_frame) override;

 private:
  /**
   * @brief load a CNN model
   */
  bool LoadCNNModel();

  /**
   * @brief load a CNN_LSTM like model
   */
  bool LoadCNNLSTMModel();

  /**
   * @brief inference a CNN model
   * @param learning_data_frame input and output intermediate for inference
   */
  bool DoCNNMODELInference(LearningDataFrame* const learning_data_frame);

  /**
   * @brief inference a CNN_LSTM like model
   * @param learning_data_frame input and output intermediate for inference
   */
  bool DoCNNLSTMMODELInference(LearningDataFrame* const learning_data_frame);

  /**
   * @brief postprocessing model trajectory output
   */
  void output_postprocessing(const at::Tensor& torch_output_tensor,
                             LearningDataFrame* const learning_data_frame);

  torch::jit::script::Module model_;
  torch::Device device_;
};

}  // namespace planning
}  // namespace apollo
