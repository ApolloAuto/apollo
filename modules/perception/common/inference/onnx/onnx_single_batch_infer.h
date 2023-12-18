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

#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#if GPU_PLATFORM == NVIDIA
#include <NvInferPlugin.h>
#include <NvInferRuntimeCommon.h>
#include <NvInferVersion.h>

#include "NvInfer.h"
#include "NvInferVersion.h"
#include "NvOnnxParser.h"
#endif

#include "modules/perception/common/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

using BlobPtr = std::shared_ptr<apollo::perception::base::Blob<float>>;

class SingleBatchInference : public Inference {
 public:
  SingleBatchInference(const std::string &model_file,
                       const float score_threshold,
                       const std::vector<std::string> &outputs,
                       const std::vector<std::string> &inputs);

  SingleBatchInference(const std::string &model_file,
                       const std::vector<std::string> &outputs,
                       const std::vector<std::string> &inputs);

  virtual ~SingleBatchInference();

  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;
  void Infer() override;
  base::BlobPtr<float> get_blob(const std::string &name) override;

 private:
  /**
   * @brief load trt cached engine to cuda memory
   *
   */
  bool LoadCache(const std::string &path);

  void init_blob(std::vector<std::string> *names);
  /**
   * @brief Convert ONNX to TensorRT model
   * @param[in] model_file ONNX model file path
   * @details Load ONNX model, and convert it to TensorRT model
   */
  bool OnnxToTRTModel(const std::string &model_file);

  std::string model_file_;
  float score_threshold_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  BlobMap blobs_;

  std::mutex mutex_;
  int bindingIndex_ = 0;

  nvinfer1::ICudaEngine *engine_;
  nvinfer1::IExecutionContext *context_;
  nvinfer1::IBuilder *builder_ = nullptr;
  nvinfer1::IBuilderConfig *builder_config_ = nullptr;
  nvinfer1::INetworkDefinition *network_ = nullptr;

  std::vector<void *> buffers_;
  cudaStream_t stream_ = 0;
  int workspaceSize_ = 1;

  int num_classes_;
  int kBatchSize;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
