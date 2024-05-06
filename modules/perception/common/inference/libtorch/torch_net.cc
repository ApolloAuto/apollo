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

#include "modules/perception/common/inference/libtorch/torch_net.h"

#include "cyber/common/log.h"
#include "modules/perception/common/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

using apollo::perception::base::Blob;

TorchNet::TorchNet(const std::string &model_file,
                   const std::vector<std::string> &outputs,
                   const std::vector<std::string> &inputs)
    : model_file_(model_file), output_names_(outputs), input_names_(inputs) {}

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

  // add blobs
  for (const auto &name : input_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      auto blob = std::make_shared<Blob<float>>(iter->second);
      blobs_.emplace(name, blob);
    }
  }

  for (const auto &name : output_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      auto blob = std::make_shared<Blob<float>>(iter->second);
      blobs_.emplace(name, blob);
    }
  }
  return true;
}

std::shared_ptr<Blob<float>> TorchNet::get_blob(const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

bool TorchNet::reshape() { return true; }

bool TorchNet::shape(const std::string &name, std::vector<int> *res) {
  auto blob = get_blob(name);
  if (blob == nullptr) {
    return false;
  }

  *res = blob->shape();
  return true;
}

void TorchNet::Infer() {
  torch::Device device(device_type_, device_id_);
  // Get input data from blob to torch_blob.
  std::vector<torch::jit::IValue> torch_inputs;
  for (const auto &name : input_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      std::vector<int64_t> shape(blob->shape().begin(), blob->shape().end());
      torch::Tensor torch_blob = torch::from_blob(
          blob->data()->mutable_cpu_data(), shape, torch::kFloat32);
      torch_blob = torch_blob.to(device);
      torch_inputs.push_back(torch_blob);
    }
  }
  // If `out_blob->mutable_cpu_data()` is invoked outside,
  // HEAD will be set to CPU, and `out_blob->mutable_gpu_data()`
  // after `enqueue` will copy data from CPU to GPU,
  // which will overwrite the `inference` results.
  // `out_blob->gpu_data()` will set HEAD to SYNCED,
  // then no copy happends after `enqueue`.
  for (const auto &name : output_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      blob->gpu_data();
    }
  }

  // Infer
  std::vector<torch::Tensor> output =
      net_.forward(torch_inputs).toTensorVector();

  // Fill output
  for (size_t i = 0; i < output_names_.size(); ++i) {
    auto blob = get_blob(output_names_[i]);
    if (blob != nullptr && i < output.size()) {
      std::vector<int64_t> output_size = output[i].sizes().vec();
      std::vector<int> shape(output_size.begin(), output_size.end());
      blob->Reshape(shape);
      blob->set_gpu_data(output[i].data_ptr<float>());
    }
  }
  emptyCache();
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
