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

#ifndef MODULES_PREDICTION_EVALUATOR_VEHICLE_RNN_EVALUATOR_H_
#define MODULES_PREDICTION_EVALUATOR_VEHICLE_RNN_EVALUATOR_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>

#include "Eigen/Dense"

#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/proto/network_model.pb.h"
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
  void Evaluate(Obstacle* obstacle_ptr) override;

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
   * @brief Clear
   */
  void Clear();

 private:
  /**
   * @brief Load mode file
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
  network::RnnModel* model_ptr_;
  int dim_obstacle_feature_ = 6;
  int dim_lane_point_feature_ = 4;
  int length_lane_point_sequence_ = 20;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_EVALUATOR_VEHICLE_RNN_EVALUATOR_H_
