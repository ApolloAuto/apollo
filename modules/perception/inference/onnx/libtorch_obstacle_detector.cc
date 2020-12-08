/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/inference/onnx/libtorch_obstacle_detector.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

using apollo::perception::base::Blob;

ObstacleDetector::ObstacleDetector(const std::string &net_file,
                                   const std::string &model_file,
                                   const std::vector<std::string> &outputs):
                                   net_file_(net_file),
                                   model_file_(model_file),
                                   output_names_(outputs) {}

bool ObstacleDetector::Init(const std::map<std::string,
                            std::vector<int>> &shapes) {
  if (gpu_id_ >= 0) {
    device_type_ = torch::kCUDA;
    device_id_ = gpu_id_;
  } else {
    device_type_ = torch::kCPU;
  }

  // Init net
  torch::Device device(device_type_, device_id_);
  net_ = torch::jit::load(model_file_, device);
  net_.eval();

  // run a fake inference at init time as first inference is relative slow
  torch::Tensor input_feature_tensor = torch::zeros({1, 3, 640, 960});
  std::array<float, 2> down_ratio{8.0f, 8.0f};
  torch::Tensor tensor_downratio = torch::from_blob(down_ratio.data(), {1, 2});
  std::array<float, 9> K{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  torch::Tensor tensor_K = torch::from_blob(K.data(), {1, 3, 3});

  std::vector<torch::jit::IValue> torch_inputs;
  // torch_inputs.push_back(std::make_tuple(input_feature_tensor.to(device),
  //                       tensor_downratio.to(device)));
  torch_inputs.push_back(input_feature_tensor.to(device));
  torch_inputs.push_back(std::make_tuple(tensor_K.to(device),
                                         tensor_downratio.to(device)));
  auto torch_output_tensor =
      net_.forward(torch_inputs).toTensor();

  for (const auto& name : output_names_) {
    auto blob = std::make_shared<Blob<float>>(2, 6, 1, 1);
    blobs_.emplace(name, blob);
  }

  for (const auto& name : input_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      auto blob = std::make_shared<Blob<float>>(iter->second);
      blobs_.emplace(name, blob);
    }
  }
  return true;
}

ObstacleDetector::ObstacleDetector(const std::string &net_file,
                   const std::string &model_file,
                   const std::vector<std::string> &outputs,
                   const std::vector<std::string> &inputs):
                   net_file_(net_file), model_file_(model_file),
                   output_names_(outputs), input_names_(inputs) {}

std::shared_ptr<Blob<float>> ObstacleDetector::get_blob(
    const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

void ObstacleDetector::Infer() {
  torch::Device device(device_type_, device_id_);
  auto blob = blobs_[input_names_[0]];
  auto input_K = blobs_[input_names_[1]];
  auto input_ratio = blobs_[input_names_[2]];

  torch::Tensor tensor_image = torch::from_blob(
                              blob->data()->mutable_gpu_data(),
                              {blob->shape(0), blob->shape(1), blob->shape(2),
                              blob->shape(3)}, torch::kFloat32);
  torch::Tensor tensor_K = torch::from_blob(
                    input_K->data()->mutable_cpu_data(),
                    {input_K->shape(0), input_K->shape(1), input_K->shape(2)},
                    torch::kFloat32);
  torch::Tensor tensor_ratio = torch::from_blob(
                    input_ratio->data()->mutable_cpu_data(),
                    {input_ratio->shape(0), input_ratio->shape(1)},
                    torch::kFloat32);

  std::vector<torch::jit::IValue> torch_inputs;
  tensor_image = tensor_image.to(device);
  tensor_image = tensor_image.permute({0, 3, 1, 2}).contiguous();

  AINFO << tensor_image[0][0].sizes();
  tensor_image[0][0] = tensor_image[0][0].div_(58.395);
  tensor_image[0][1] = tensor_image[0][1].div_(57.12);
  tensor_image[0][2] = tensor_image[0][2].div_(57.375);
  AINFO << tensor_K[0][0];

  torch_inputs.push_back(tensor_image);
  torch_inputs.push_back(std::make_tuple(tensor_K.to(device),
                                         tensor_ratio.to(device)));

  AINFO << "Start to do inference";
  auto outputs = net_.forward(torch_inputs).toTensor();
  AINFO << "Finished inference";
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
