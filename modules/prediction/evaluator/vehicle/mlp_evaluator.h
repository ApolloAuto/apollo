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

#ifndef MODULES_PREDICTION_EVALUATOR_VEHICLE_MLP_EVALUATOR_H_
#define MODULES_PREDICTION_EVALUATOR_VEHICLE_MLP_EVALUATOR_H_

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>

#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/proto/lane_graph.pb.h"
#include "modules/prediction/proto/fnn_vehicle_model.pb.h"

namespace apollo {
namespace prediction {

class MLPEvaluator : public Evaluator {
 public:
  MLPEvaluator() = default;

  virtual ~MLPEvaluator() = default;

  void Evaluate(Obstacle* obstacle_ptr);

  void ExtractFeatureValues(Obstacle* obstacle_ptr,
                            LaneSequence* lane_sequence_ptr);

  void Predict(Obstacle* obstacle_ptr);

  void Clear();

 private:
  void SetObstacleFeatureValues(Obstacle* obstacle_ptr,
                                std::vector<double>* feature_values);

  void SetLaneFeatureValues(Obstacle* obstacle_ptr,
                            LaneSequence* lane_sequence_ptr,
                            std::vector<double>* feature_values);

  void LoadModel(const std::string& model_file);

  double ComputeProbability();

 private:
  std::unordered_map<int, std::vector<double>> obstacle_feature_values_map_;
  static const size_t OBSTACLE_FEATURE_SIZE = 18;
  static const size_t LANE_FEATURE_SIZE = 20;

  std::unique_ptr<FnnVehicleModel> model_ptr_;
};

}  // namespace prediction
}  // namespace apollo

#endif  // MODULES_PREDICTION_EVALUATOR_VEHICLE_MLP_EVALUATOR_H_
