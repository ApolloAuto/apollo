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

#include "modules/prediction/evaluator/model_manager/model/multi_agent_vehicle_torch_cpu/multi_agent_vehicle_torch_model.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/prediction/proto/prediction_conf.pb.h"

#include "cyber/common/file.h"
#include "modules/prediction/evaluator/warm_up/warm_up.h"

namespace apollo {
namespace prediction {

bool MultiAgentVehicleCpuTorch::Init() {
  ModelConf model_config;
  int status;

  if (init_ != 0) {
    return true;
  }

  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);

  std::string default_config_path =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->GetPluginConfPath<ModelBase>(class_name,
                                         "conf/default_conf.pb.txt");

  if (!cyber::common::GetProtoFromFile(default_config_path, &model_config)) {
    AERROR << "Unable to load model conf file: " << default_config_path;
    return false;
  }
  model_path_ = model_config.model_path();
  init_ = 1;

  return LoadModel();
}

bool MultiAgentVehicleCpuTorch::LoadModel() {
  auto device = torch::Device(torch::kCPU);
  if (torch::cuda::is_available()) {
    device = torch::Device(torch::kCUDA);
  }

  model_instance_ = torch::jit::load(model_path_, device);

  torch::set_num_threads(1);

  // Fake intput for the first frame
  torch::Tensor target_obstacle_pos =
    torch::randn({1, max_agent_num, 20, 2});
  torch::Tensor target_obstacle_pos_step =
    torch::randn({1, max_agent_num, 20, 2});
  torch::Tensor vector_data = torch::randn({1, 450, 50, 9});
  torch::Tensor vector_mask = torch::randn({1, 450, 50}) > 0.9;
  torch::Tensor polyline_mask = torch::randn({1, 450}) > 0.9;
  torch::Tensor rand_mask = torch::zeros({450}).toType(at::kBool);
  torch::Tensor polyline_id = torch::randn({1, 450, 2});
  torch::Tensor obs_position = torch::randn({1, max_agent_num, 3});
  std::vector<torch::jit::IValue> torch_inputs;
  torch::Tensor torch_default_output_tensor;

  torch_inputs.push_back(c10::ivalue::Tuple::create(
      {std::move(target_obstacle_pos.to(device)),
       std::move(target_obstacle_pos_step.to(device)),
       std::move(vector_data.to(device)),
       std::move(vector_mask.to(device)),
       std::move(polyline_mask.to(device)),
       std::move(rand_mask.to(device)),
       std::move(polyline_id.to(device)),
       std::move(obs_position.to(device))}));

  // warm up to avoid very slow first inference later
  WarmUp(torch_inputs, &model_instance_, &torch_default_output_tensor);
  return true;
}

bool MultiAgentVehicleCpuTorch::Inference(
    const std::vector<void*>& input_buffer, unsigned int input_size,
    std::vector<void*>* output_buffer, unsigned int output_size) {
  ACHECK(input_size == input_buffer.size() && input_size == 8);
  ACHECK(output_size == output_buffer->size() && output_size == 1);

  if (init_ == 0) {
    Init();
  }

  auto device = torch::Device(torch::kCPU);
  if (torch::cuda::is_available()) {
    device = torch::Device(torch::kCUDA);
  }
  torch::Tensor target_obstacle_pos =
      torch::from_blob(input_buffer[0], {1, max_agent_num, 20, 2});
  torch::Tensor target_obstacle_pos_step =
      torch::from_blob(input_buffer[1], {1, max_agent_num, 20, 2});
  torch::Tensor vector_data =
      torch::from_blob(input_buffer[2], {1, 450, 50, 9});
  auto options = torch::TensorOptions().dtype(torch::kBool);
  torch::Tensor vector_mask =
      torch::from_blob(input_buffer[3], {1, 450, 50}, options);
  torch::Tensor polyline_mask =
      torch::from_blob(input_buffer[4], {1, 450}, options);
  torch::Tensor rand_mask =
      torch::from_blob(input_buffer[5], {1, 450}, options);
  torch::Tensor polyline_id =
      torch::from_blob(input_buffer[6], {1, 450, 2});
  torch::Tensor obs_position =
      torch::from_blob(input_buffer[7], {1, max_agent_num, 3});

  std::vector<torch::jit::IValue> torch_inputs;

  torch_inputs.push_back(c10::ivalue::Tuple::create(
      {std::move(target_obstacle_pos.to(device)),
       std::move(target_obstacle_pos_step.to(device)),
       std::move(vector_data.to(device)),
       std::move(vector_mask.to(device)),
       std::move(polyline_mask.to(device)),
       std::move(rand_mask.to(device)),
       std::move(polyline_id.to(device)),
       std::move(obs_position.to(device))}));

  torch::Tensor torch_output_tensor =
      model_instance_.forward(torch_inputs).toTensor().to(torch::kCPU);
  memcpy((*output_buffer)[0], torch_output_tensor.data_ptr<float>(),
         1 * max_agent_num * 30 * 2 * sizeof(float));
  return true;
}

void MultiAgentVehicleCpuTorch::Destory() {}

}  // namespace prediction
}  // namespace apollo
