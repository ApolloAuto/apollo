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
#include "modules/prediction/proto/fnn_vehicle_model.pb.h"

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
  void Evaluate(Obstacle* obstacle_ptr) override;

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
  std::string GetName() override {return "JUNCTION_MLP_EVALUATOR";}

 private:
  /**
   * @brief Set obstacle feature vector
   * @param Obstacle pointer
   *        Feature container in a vector for receiving the feature values
   */
  void SetObstacleFeatureValues(Obstacle* obstacle_ptr,
                                std::vector<double>* feature_values);

  /**
   * @brief Set junction feature vector
   * @param Obstacle pointer
   *        Feature container in a vector for receiving the feature values
   */
  void SetJunctionFeatureValues(Obstacle* obstacle_ptr,
                                std::vector<double>* feature_values);

  /**
   * @brief Save offline feature values in proto
   * @param Obstacle feature
   * @param Vector of feature values
   */
  void SaveOfflineFeatures(Feature* feature_ptr,
                           const std::vector<double>& feature_values);

  /**
   * @brief Load mode file
   * @param Model file name
   */
  void LoadModel(const std::string& model_file);

  /**
   * @brief Compute probability of a junction exit
   */
  std::vector<double> ComputeProbability(
      const std::vector<double>& feature_values);

 private:
  static const size_t OBSTACLE_FEATURE_SIZE = 3;
  static const size_t JUNCTION_FEATURE_SIZE = 60;

  std::unique_ptr<FnnVehicleModel> model_ptr_;
};

}  // namespace prediction
}  // namespace apollo
