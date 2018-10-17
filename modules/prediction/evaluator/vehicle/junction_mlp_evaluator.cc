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

#include "modules/prediction/evaluator/vehicle/junction_mlp_evaluator.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

JunctionMLPEvaluator::JunctionMLPEvaluator() {
  LoadModel(FLAGS_evaluator_vehicle_junction_mlp_file);
}

void JunctionMLPEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // TODO(all) implement
  // 1. extract features
  // 2. compute probabilities
}

void JunctionMLPEvaluator::ClusterLaneIds(Feature* feature_ptr) {
  // TODO(all) implement
  // group lane ids belonging to the same junction exit
}

void JunctionMLPEvaluator::LoadModel(const std::string& model_file) {
  // TODO(all) implement
  // 1. Make model file ready
  // 2. Load model from file
}

double JunctionMLPEvaluator::ComputeProbability(
    const std::vector<double>& feature_values) {
  // TODO(all) implement
  return 0.5;
}

}  // namespace prediction
}  // namespace apollo
