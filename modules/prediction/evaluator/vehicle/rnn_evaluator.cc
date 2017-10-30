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

#include "modules/prediction/evaluator/vehicle/rnn_evaluator.h"

#include "modules/common/util/file.h"
#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

RNNEvaluator::RNNEvaluator() { LoadModel(FLAGS_vehicle_model_file); }

void RNNEvaluator::Evaluate(Obstacle* obstacle_ptr) {
  Clear();
  CHECK_NOTNULL(obstacle_ptr);
}

void RNNEvaluator::Clear() {}

void RNNEvaluator::LoadModel(const std::string& model_file) {
  model_ptr_.reset(new NetParameter());
  CHECK(model_ptr_ != nullptr);
  CHECK(common::util::GetProtoFromFile(model_file, model_ptr_.get()))
      << "Unable to load model file: " << model_file << ".";

  AINFO << "Succeeded in loading the model file: " << model_file << ".";
}

}  // namespace prediction
}  // namespace apollo
