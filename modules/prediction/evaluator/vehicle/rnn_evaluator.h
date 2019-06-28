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

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/network/rnn_model/rnn_model.h"

namespace apollo {
namespace prediction {

class RNNEvaluator : public Evaluator {
 public:
  /**
   * @brief Constructor
   */
  RNNEvaluator();

  /**
   * @brief Destructor
   */
  virtual ~RNNEvaluator() = default;

  /**
   * @brief Override Evaluate
   * @param Obstacle pointer
   */
  bool Evaluate(Obstacle* obstacle_ptr) override;

  /**
   * @brief Extract feature vector
   * @param obstacle a pointer to the target obstacle
   * @param obstacle_feature_mat feature matrix
   * @param lane_feature_mats lane feature matrices
   */
  int ExtractFeatureValues(
      Obstacle* obstacle, Eigen::MatrixXf* const obstacle_feature_mat,
      std::unordered_map<int, Eigen::MatrixXf>* const lane_feature_mats);

  /**
   * @brief Get the name of evaluator.
   */
  std::string GetName() override { return "RNN_EVALUATOR"; }

  /**
   * @brief Clear
   */
  void Clear();

 private:
  /**
   * @brief Load model file
   * @param Model file name
   */
  void LoadModel(const std::string& model_file);

  int SetupObstacleFeature(Obstacle* obstacle,
                           std::vector<float>* const feature_values);

  int SetupLaneFeature(const Feature& feature,
                       const LaneSequence& lane_sequence,
                       std::vector<float>* const feature_values);

  bool IsCutinInHistory(const std::string& curr_lane_id,
                        const std::string& prev_lane_id);

 private:
  static const int DIM_OBSTACLE_FEATURE = 6;
  static const int DIM_LANE_POINT_FEATURE = 4;
  static const int LENGTH_LANE_POINT_SEQUENCE = 20;
  network::RnnModel* model_ptr_ = nullptr;
};

}  // namespace prediction
}  // namespace apollo
