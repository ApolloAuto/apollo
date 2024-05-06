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

/**
 * @file
 * @brief warm-up function for torch model to avoid first multiple slowly inference
 */

#pragma once

#include <vector>

#include "torch/extension.h"
#include "torch/script.h"

/**
 * @namespace apollo::prediction
 * @brief apollo::prediction
 */
namespace apollo {
namespace prediction {

/**
   * @brief warm up function to avoid slowly inference of torch model
   * @param torch_inputs input vector of torch model
   * @param model torch model instance
   * @param default_output pointer of output, which is usually maintained by evaluator
   */
void WarmUp(const std::vector<torch::jit::IValue>& torch_inputs,
            torch::jit::script::Module* model,
            at::Tensor* default_output);


}  // namespace prediction
}  // namespace apollo
