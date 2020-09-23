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

#include "modules/perception/inference/libtorch/torch_det.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

using apollo::perception::base::Blob;

TorchDet::TorchDet(const std::string &net_file,
      const std::string &model_file, const std::vector<std::string> &outputs)
    : net_file_(net_file), model_file_(model_file), output_names_(outputs) {}

bool TorchDet::Init(const std::map<std::string, std::vector<int>> &shapes) {
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

TorchDet::TorchDet(const std::string &net_file,
                   const std::string &model_file,
                   const std::vector<std::string> &outputs,
                   const std::vector<std::string> &inputs)
    : net_file_(net_file),
      model_file_(model_file),
      output_names_(outputs),
      input_names_(inputs) {}

std::shared_ptr<Blob<float>> TorchDet::get_blob(
    const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

void TorchDet::Infer() {
  torch::Device device(device_type_, device_id_);
  auto blob = blobs_[input_names_[0]];
  auto input_param = blobs_[input_names_[1]];

  torch::Tensor tensor_image = torch::from_blob(
                              blob->data()->mutable_gpu_data(),
                              {blob->shape(0), blob->shape(1), blob->shape(2),
                              blob->shape(3)}, torch::kFloat32);
  torch::Tensor tensor_param = torch::from_blob(
                              input_param->data()->mutable_cpu_data(),
                              {input_param->shape(0), input_param->shape(2)},
                              torch::kFloat32);

  if (device_id_ >= 0) {
    tensor_image = tensor_image.to(device);
    tensor_param = tensor_param.to(device);
  }

  tensor_image = tensor_image.permute({0, 3, 1, 2}).contiguous();
  auto outputs = net_.forward(
    {std::make_tuple(tensor_image, tensor_param)}).toTuple()->elements();

  torch::Tensor bbox = outputs[0].toTensor();
  torch::Tensor scores = outputs[1].toTensor().unsqueeze(1);
  torch::Tensor labels = outputs[2].toTensor().unsqueeze(1);
  torch::Tensor img_id = torch::zeros({scores.sizes()},
                                      torch::kFloat).to(device);
  torch::Tensor one_hot = torch::zeros({scores.size(0), 2},
                                       torch::kFloat).to(device);

  torch::Tensor result = torch::cat({img_id, bbox, labels, scores, one_hot},
                                   1);
  blobs_[output_names_[0]]->Reshape({static_cast<int>(result.size(0)),
                             static_cast<int>(result.size(1)), 1, 1});
  if (result.size(0) == 0) {
    result = torch::zeros({1, 9}, torch::kFloat).to(device);
  }
  blobs_[output_names_[0]]->data()->set_gpu_data(result.data_ptr());
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
