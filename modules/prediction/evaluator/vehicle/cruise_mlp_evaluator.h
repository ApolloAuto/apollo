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

#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/network/cruise_model/cruise_model.h"

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
   */
  void Evaluate(Obstacle* obstacle_ptr) override;

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
  std::string GetName() override {return "CRUISE_MLP_EVALUATOR";}

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
   *        Lane sequence pointer
   *        Feature container in a vector for receiving the feature values
   */
  void SetInteractionFeatureValues(Obstacle* obstacle_ptr,
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
   * @brief Load mode files
   * @param Go model file name
   * @param Cutin model file name
   */
  void LoadModels(const std::string& go_model_file,
                  const std::string& cutin_model_file);

  /**
   * @brief Compute probability of a junction exit
   */
  double ComputeFinishTime(const std::vector<double>& feature_values);

  /**
   * @brief Save offline feature values in proto
   * @param Lane sequence
   * @param Vector of feature values
   */
  void SaveOfflineFeatures(LaneSequence* sequence,
                           const std::vector<double>& feature_values);

 private:
  static const size_t OBSTACLE_FEATURE_SIZE = 23 + 5 * 9;
  static const size_t INTERACTION_FEATURE_SIZE = 8;
  static const size_t SINGLE_LANE_FEATURE_SIZE = 4;
  static const size_t LANE_POINTS_SIZE = 20;

  std::shared_ptr<network::CruiseModel> go_model_ptr_;
  std::shared_ptr<network::CruiseModel> cutin_model_ptr_;
};

}  // namespace prediction
}  // namespace apollo
