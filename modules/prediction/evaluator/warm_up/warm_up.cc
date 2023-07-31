/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/prediction/evaluator/warm_up/warm_up.h"

#include "modules/prediction/common/prediction_gflags.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

void WarmUp(const std::vector<torch::jit::IValue>& torch_inputs,
            torch::jit::script::Module* model,
            at::Tensor* default_output_ptr) {
  for (uint32_t i = 0; i < FLAGS_warm_up_times; i++) {
    if (default_output_ptr != nullptr)
      *default_output_ptr =
          model->forward(torch_inputs).toTensor().to(torch::kCPU);
    else
      model->forward(torch_inputs);
  }
}

}  // namespace prediction
}  // namespace apollo
