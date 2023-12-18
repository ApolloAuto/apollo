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

#pragma once

#include <vector>

#include "torch/extension.h"
#include "torch/script.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/prediction/evaluator/model_manager/model/model_base.h"

namespace apollo {
namespace prediction {

class SemanticLstmVehicleGpuTorch : public ModelBase {
 public:
  SemanticLstmVehicleGpuTorch() {}
  ~SemanticLstmVehicleGpuTorch() {Destory();}

  /**
   * @brief parse model description class and load the model
   *
   * @param config_path model config path
   * @return init result, true for success
   */
  virtual bool Init();

  /**
   * @brief performing network inference
   *
   * @param input_buffer vector of input tensor
   * @param input_size size of input_buffer
   * @param output_buffer vector of output tensor
   * @param output_size size of output_buffer
   * @return init result, true for success
   */
  virtual bool Inference(const std::vector<void*>& input_buffer,
                         unsigned int input_size,
                         std::vector<void*>* output_buffer,
                         unsigned int output_size);

  /**
   * @brief load the model from file
   *
   * @return loading result, true for success
   */
  virtual bool LoadModel();

  /**
   * @brief free all memory requested, gpu or cpu
   *
   * @return memory release result, true for success
   */ 
  virtual void Destory();

 private:
  torch::jit::script::Module model_instance_;
};
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
  apollo::prediction::SemanticLstmVehicleGpuTorch, ModelBase)

}  // namespace prediction
}  // namespace apollo
