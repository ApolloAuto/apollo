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

#include "modules/perception/inference/libtorch/torch_net.h"
#include "modules/perception/inference/inference.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

using apollo::perception::base::Blob;

TorchNet::TorchNet(const std::string &net_file, const std::string &model_file,
                   const std::vector<std::string> &outputs)
    : net_file_(net_file), model_file_(model_file), output_names_(outputs) {}

bool TorchNet::Init(const std::map<std::string, std::vector<int>> &shapes) {
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

  for (auto name : output_names_) {
    auto blob = std::make_shared<Blob<float>>(1, 4, 1, 1);
    blobs_.emplace(name, blob);
  }

  for (auto name : input_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      auto blob = std::make_shared<Blob<float>>(iter->second);
      blobs_.emplace(name, blob);
    }
  }
  return true;
}

TorchNet::TorchNet(const std::string &net_file, const std::string &model_file,
                   const std::vector<std::string> &outputs,
                   const std::vector<std::string> &inputs)
    : net_file_(net_file),
      model_file_(model_file),
      output_names_(outputs),
      input_names_(inputs) {}

std::shared_ptr<Blob<float>> TorchNet::get_blob(const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

bool TorchNet::reshape() {
  return true;
}

void TorchNet::Infer() {
  torch::Device device(device_type_, device_id_);
  auto blob = blobs_[input_names_[0]];

  // pay attention to the tensor shape order, if changed without permute
  // will get wrong result
  torch::Tensor tensor_image = torch::from_blob(
                              blob->data()->mutable_gpu_data(),
                              {blob->shape(1), blob->shape(2), blob->shape(3)},
                              torch::kFloat32);
  if (device_id_ >= 0) {
    tensor_image = tensor_image.to(device);
  }

  tensor_image = tensor_image.permute({2, 0, 1});
  tensor_image = tensor_image.toType(torch::kFloat32);
  tensor_image[0] = tensor_image[0].div_(58.395);
  tensor_image[1] = tensor_image[1].div_(57.12);
  tensor_image[2] = tensor_image[2].div_(57.375);
  tensor_image = tensor_image.unsqueeze(0);

  torch::Tensor output = net_.forward({tensor_image}).toTensor();
  torch::Tensor prob = torch::softmax(output, 1);
  blobs_[output_names_[0]]->data()->set_gpu_data(prob.data_ptr());
  emptyCache();
}


}  // namespace inference
}  // namespace perception
}  // namespace apollo
