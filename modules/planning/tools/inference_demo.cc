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

#include <gflags/gflags.h>

#include "torch/script.h"
#include "torch/torch.h"

DEFINE_string(model_file,
              "/apollo/modules/planning/tools/planning_demo_model.pt",
              "pytorch model file.");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  torch::jit::script::Module model;
  std::cout << "is_optimized:" << model.is_optimized() << std::endl;
  std::cout << "parameter size:" << model.get_parameters().size() << std::endl;

  torch::Device device(torch::kCPU);

  torch::set_num_threads(1);
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    model = torch::jit::load(FLAGS_model_file, device);
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return -1;
  }
  std::cout << "is_optimized:" << model.is_optimized() << std::endl;
  std::cout << "after loading parameter size:"
            << model.get_parameters().size() << std::endl;

  std::vector<torch::jit::IValue> torch_inputs;

  int input_dim = 2 * 3 * 224 * 224 + 2 * 14;
  std::vector<double> feature_values(input_dim, 0.5);

  std::vector<torch::jit::IValue> inputs;
  std::vector<torch::jit::IValue> tuple;
  tuple.push_back(torch::zeros({2, 3, 224, 224}));
  tuple.push_back(torch::zeros({2, 14}));
  inputs.push_back(torch::ivalue::Tuple::create(tuple));

  auto torch_output = model.forward(inputs);
  std::cout << torch_output << std::endl;
  std::cout << "isDoubleList:" << torch_output.isDoubleList() << std::endl;
  std::cout << "isTensorList:" << torch_output.isTensorList() << std::endl;
  std::cout << "isTensor:" << torch_output.isTensor() << std::endl;
  auto torch_output_tensor = torch_output.toTensor();
  std::cout << "tensor dim:" << torch_output_tensor.dim() << std::endl;
  std::cout << "tensor sizes:" << torch_output_tensor.sizes() << std::endl;
  std::cout << "tensor toString:" << torch_output_tensor.toString()
            << std::endl;
  std::cout << "tensor [0,0,0] element:" << torch_output_tensor[0][0][0]
            << std::endl;
  std::cout << "tensor [0,0,1] element:" << torch_output_tensor[0][0][1]
            << std::endl;
  std::cout << "tensor [0,0,0] element:"
            << double(torch_output_tensor.accessor<float, 3>()[0][0][0])
            << std::endl;
  std::cout << "tensor [0,0,1] element:"
            << double(torch_output_tensor.accessor<float, 3>()[0][0][1])
            << std::endl;

  return 0;
}
