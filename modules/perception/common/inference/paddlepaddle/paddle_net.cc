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

#include "modules/perception/common/inference/paddlepaddle/paddle_net.h"

#include <algorithm>
#include <numeric>

namespace apollo {
namespace perception {
namespace inference {

PaddleNet::PaddleNet(const std::string& model_file,
                     const std::string& params_file,
                     const std::vector<std::string>& outputs,
                     const std::vector<std::string>& inputs)
    : model_file_(model_file),
      params_file_(params_file),
      output_names_(outputs),
      input_names_(inputs) {}

PaddleNet::~PaddleNet() {}

bool PaddleNet::Init(const std::map<std::string, std::vector<int>>& shapes) {
  paddle_infer::Config config;
  config.SetModel(model_file_, params_file_);

  if (gpu_id_ >= 0) {
    config.EnableUseGpu(MemoryPoolInitSizeMb, gpu_id_);
  }

  if (FLAGS_use_trt) {
    paddle::AnalysisConfig::Precision precision;
    if (FLAGS_trt_precision == 0) {
      precision = paddle_infer::PrecisionType::kFloat32;
    } else if (FLAGS_trt_precision == 1) {
      precision = paddle_infer::PrecisionType::kInt8;
    } else if (FLAGS_trt_precision == 2) {
      precision = paddle_infer::PrecisionType::kHalf;
    } else {
      AERROR << "Tensorrt type can only support 0, 1, 2, but recieved is"
             << FLAGS_trt_precision << "\n";
      return false;
    }
    config.EnableTensorRtEngine(1 << 30, 1, 3, precision, FLAGS_trt_use_static,
                                FLAGS_use_calibration);

    if (FLAGS_collect_shape_info) {
      config.CollectShapeRangeInfo(FLAGS_dynamic_shape_file);
    }

    if (FLAGS_use_dynamicshape)
      config.EnableTunedTensorRtDynamicShape(FLAGS_dynamic_shape_file, true);
  }

  config.EnableMemoryOptim();
  config.SwitchIrOptim(true);
  predictor_ = paddle_infer::CreatePredictor(config);
  if (predictor_ == nullptr) {
    return false;
  }

  // Check input and output
  auto input_names = predictor_->GetInputNames();
  auto output_names = predictor_->GetOutputNames();

  std::vector<std::string> exist_names;
  for (const std::string& name : input_names_) {
    if (std::find(input_names.begin(), input_names.end(), name) !=
        input_names.end()) {
      exist_names.push_back(name);
    }
  }
  input_names_ = exist_names;

  exist_names.clear();
  for (const std::string& name : output_names_) {
    if (std::find(output_names.begin(), output_names.end(), name) !=
        output_names.end()) {
      exist_names.push_back(name);
    }
  }
  output_names_ = exist_names;

  // add blobs
  for (const auto& shape : shapes) {
    auto blob =
        std::make_shared<apollo::perception::base::Blob<float>>(shape.second);
    blobs_.emplace(shape.first, blob);
  }

  return true;
}

void PaddleNet::Infer() {
  // reshape and get input data from blob to paddle_blob.
  for (const auto& name : input_names_) {
    auto blob = get_blob(name);
    auto paddle_blob = predictor_->GetInputHandle(name);
    if (paddle_blob != nullptr && blob != nullptr) {
      paddle_blob->Reshape(blob->shape());
      paddle_blob->CopyFromCpu(blob->cpu_data());
    }
  }
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
      // label : int64_t
      if (paddle_blob->type() == paddle_infer::INT64) {
        ACHECK(1 == paddle_blob_shape.size());
        std::vector<int64_t> label_i(paddle_blob_shape.at(0));
        paddle_blob->CopyToCpu(label_i.data());
        std::vector<float> label_f(label_i.data(),
                                   label_i.data() + paddle_blob_shape.at(0));
        memcpy(blob->mutable_cpu_data(), label_f.data(),
               paddle_blob_shape.at(0) * sizeof(float));
      } else {
        paddle_blob->CopyToCpu(blob->mutable_cpu_data());
      }
    }
  }
}

bool PaddleNet::shape(const std::string& name, std::vector<int>* res) {
  auto blob = get_blob(name);
  if (blob == nullptr) {
    return false;
  }

  *res = blob->shape();
  return true;
}

base::BlobPtr<float> PaddleNet::get_blob(const std::string& name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
