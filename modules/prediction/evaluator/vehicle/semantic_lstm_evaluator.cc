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

#include "modules/prediction/evaluator/vehicle/semantic_lstm_evaluator.h"

#include <omp.h>
#include <unordered_map>
#include <utility>

#include "cyber/common/file.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"
#include "modules/prediction/common/semantic_map.h"

namespace apollo {
namespace prediction {

SemanticLSTMEvaluator::SemanticLSTMEvaluator() : device_(torch::kCPU) {
  evaluator_type_ = ObstacleConf::SEMANTIC_LSTM_EVALUATOR;
  LoadModel();
}

void SemanticLSTMEvaluator::Clear() {}

bool SemanticLSTMEvaluator::Evaluate(Obstacle* obstacle_ptr) { return true; }

bool SemanticLSTMEvaluator::ExtractFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
  return true;
}

void SemanticLSTMEvaluator::LoadModel() {
  if (FLAGS_use_cuda && torch::cuda::is_available()) {
    ADEBUG << "CUDA is available";
    device_ = torch::Device(torch::kCUDA);
  }
  torch::set_num_threads(1);
  // TODO(Hongyi): change model file name and gflag
  torch_model_ =
      torch::jit::load(FLAGS_torch_vehicle_junction_map_file, device_);
}

}  // namespace prediction
}  // namespace apollo
