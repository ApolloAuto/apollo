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
#include <string>
#include <utility>
#include <vector>

#if GPU_PLATFORM == NVIDIA
  #include "NvInfer.h"
  #include "NvOnnxParser.h"
  #include "NvInferVersion.h"
#endif

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/inference/inference.h"

namespace apollo {
namespace perception {
namespace inference {

using BlobPtr = std::shared_ptr<apollo::perception::base::Blob<float>>;

// Logger for TensorRT info/warning/errors
#if GPU_PLATFORM == NVIDIA
class Logger : public nvinfer1::ILogger {
 public:
  explicit Logger(Severity severity = Severity::kWARNING)
      : reportable_severity(severity) {}

  void log(Severity severity, const char* msg) override {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportable_severity) return;

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        std::cerr << "INTERNAL_ERROR: ";
        break;
      case Severity::kERROR:
        std::cerr << "ERROR: ";
        break;
      case Severity::kWARNING:
        std::cerr << "WARNING: ";
        break;
      case Severity::kINFO:
        std::cerr << "INFO: ";
        break;
      default:
        std::cerr << "UNKNOWN: ";
        break;
    }
    std::cerr << msg << std::endl;
  }

  Severity reportable_severity;
};
#elif GPU_PLATFORM == AMD
  class Logger {};
namespace nvinfer1 {
  class ICudaEngine {};
  class IExecutionContext {};
}  // namespace nvinfer1
#endif

class OnnxObstacleDetector : public Inference {
 public:
  OnnxObstacleDetector(const std::string &model_file,
                       const float score_threshold,
                       const std::vector<std::string> &outputs,
                       const std::vector<std::string> &inputs);

  OnnxObstacleDetector(const std::string &model_file,
                       const std::vector<std::string> &outputs,
                       const std::vector<std::string> &inputs);

  virtual ~OnnxObstacleDetector();

  /**
   * @brief Convert ONNX to TensorRT model
   * @param[in] model_file ONNX model file path
   * @param[out] engine_ptr TensorRT model engine made out of ONNX model
   * @details Load ONNX model, and convert it to TensorRT model
   */
  void OnnxToTRTModel(const std::string& model_file,
                      nvinfer1::ICudaEngine** engine_ptr);

  void inference();

  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;
  void Infer() override;
  BlobPtr get_blob(const std::string &name) override;

 private:
  std::string model_file_;
  float score_threshold_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  BlobMap blobs_;
  nvinfer1::ICudaEngine* engine_;
  nvinfer1::IExecutionContext* context_;
  Logger g_logger_;

  int num_classes_;
  int kBatchSize;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
