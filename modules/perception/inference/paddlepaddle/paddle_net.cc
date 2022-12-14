/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include <numeric>

#include "modules/perception/inference/paddlepaddle/paddle_net.h"


namespace apollo {
namespace perception {
namespace inference {

PaddleNet::PaddleNet(const std::string& model_file,
            const std::string& params_file,
            const std::vector<std::string>& outputs,
            const std::vector<std::string>& inputs)
            : model_file_(model_file), params_file_(params_file),
              output_names_(outputs), input_names_(inputs) {}

PaddleNet::~PaddleNet() {}

bool PaddleNet::Init(const std::map<std::string, std::vector<int>>& shapes) {
  paddle::AnalysisConfig config;
  config.SetModel(model_file_, params_file_);

  if (gpu_id_ >= 0) {
    config.EnableUseGpu(MemoryPoolInitSizeMb, gpu_id_);
  }

  config.SwitchIrOptim(true);
  predictor_ = paddle_infer::CreatePredictor(config);
  if (predictor_ == nullptr) {
    return false;
  }

  for (const auto& shape : shapes) {
    auto blob =
        std::make_shared<apollo::perception::base::Blob<float>>(shape.second);
    blobs_.emplace(shape.first, blob);
  }

  return true;
}

void PaddleNet::Infer() {
  // reshape and get input data from blob to paddle_blob.
  reshape();
  // If `out_blob->mutable_cpu_data()` is invoked outside,
  // HEAD will be set to CPU, and `out_blob->mutable_gpu_data()`
  // after `enqueue` will copy data from CPU to GPU,
  // which will overwrite the `inference` results.
  // `out_blob->gpu_data()` will set HEAD to SYNCED,
  // then no copy happends after `enqueue`.
  for (const auto& name : output_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      blob->gpu_data();
    }
  }

  // Infer
  predictor_->Run();

  // Fill output
  for (const auto& name : output_names_) {
    auto blob = get_blob(name);
    auto paddle_blob = predictor_->GetOutputHandle(name);
    if (blob != nullptr && paddle_blob != nullptr) {
      std::vector<int> paddle_blob_shape = paddle_blob->shape();
      blob->Reshape(paddle_blob_shape);
      int count = std::accumulate(paddle_blob_shape.begin(),
          paddle_blob_shape.end(), 1, std::multiplies<int>());
      cudaMemcpy(blob->mutable_gpu_data(),
        paddle_blob->mutable_data<float>(paddle::PaddlePlace::kGPU),
        count * sizeof(float), cudaMemcpyDeviceToDevice);
    }
  }
}

bool PaddleNet::reshape() {
  for (const auto& name : input_names_) {
    auto blob = get_blob(name);
    auto paddle_blob = predictor_->GetInputHandle(name);
    if (paddle_blob != nullptr && blob != nullptr) {
      paddle_blob->Reshape(blob->shape());
      std::vector<int> paddle_blob_shape = paddle_blob->shape();
      int count =
          std::accumulate(paddle_blob_shape.begin(), paddle_blob_shape.end(), 1,
                          std::multiplies<int>());
      cudaMemcpy(paddle_blob->mutable_data<float>(paddle::PaddlePlace::kGPU),
                 blob->gpu_data(), count * sizeof(float),
                 cudaMemcpyDeviceToDevice);
    }
  }

  return true;
}

bool PaddleNet::shape(const std::string& name, std::vector<int>* res) {
  auto blob = get_blob(name);
  if (blob == nullptr) {
    return false;
  }

  *res = blob->shape();
  return true;
}

std::shared_ptr<apollo::perception::base::Blob<float>> PaddleNet::get_blob(
    const std::string& name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}


}  // namespace inference
}  // namespace perception
}  // namespace apollo
