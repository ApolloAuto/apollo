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

#include "modules/prediction/evaluator/vehicle/junction_map_evaluator.h"

#include <omp.h>

#include "cyber/common/file.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_map.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/prediction_util.h"

namespace apollo {
namespace prediction {

JunctionMAPEvaluator::JunctionMAPEvaluator() : device_(torch::kCPU) {
  LoadModel();
}

void JunctionMAPEvaluator::Clear() {}

bool JunctionMAPEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  // Sanity checks.
  omp_set_num_threads(1);
  Clear();
  CHECK_NOTNULL(obstacle_ptr);
  return true;
}

void JunctionMAPEvaluator::ExtractFeatureValues(
    Obstacle* obstacle_ptr, std::vector<double>* feature_values) {
}

void JunctionMAPEvaluator::LoadModel() {
  // TODO(all) uncomment the following when cuda issue is resolved
  // if (torch::cuda::is_available()) {
  //   ADEBUG << "CUDA is available";
  //   device_ = torch::Device(torch::kCUDA);
  // }
  torch::set_num_threads(1);
  torch_model_ptr_ =
      torch::jit::load(FLAGS_torch_vehicle_junction_mlp_file, device_);
}

}  // namespace prediction
}  // namespace apollo
