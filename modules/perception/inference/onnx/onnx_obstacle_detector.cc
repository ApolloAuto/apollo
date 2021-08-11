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

#include "modules/perception/inference/onnx/onnx_obstacle_detector.h"

#include <cuda_runtime_api.h>
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

using apollo::perception::base::Blob;

#define GPU_CHECK(ans) \
  { GPUAssert((ans), __FILE__, __LINE__); }
inline void GPUAssert(cudaError_t code, const char* file, int line,
                      bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
            line);
    if (abort) exit(code);
  }
}

OnnxObstacleDetector::OnnxObstacleDetector(
  const std::string &model_file,
  const float score_threshold,
  const std::vector<std::string> &outputs,
  const std::vector<std::string> &inputs)
  : model_file_(model_file),
    score_threshold_(score_threshold),
    output_names_(outputs),
    input_names_(inputs) {}

OnnxObstacleDetector::OnnxObstacleDetector(
  const std::string &model_file,
  const std::vector<std::string> &outputs,
  const std::vector<std::string> &inputs)
  : model_file_(model_file), output_names_(outputs), input_names_(inputs) {}

OnnxObstacleDetector::~OnnxObstacleDetector() {}

void OnnxObstacleDetector::OnnxToTRTModel(
    const std::string& model_file,  // name of the onnx model
    nvinfer1::ICudaEngine** engine_ptr) {
  int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kWARNING);
  kBatchSize = 1;

  // create the builder
  const auto explicit_batch =
      static_cast<uint32_t>(kBatchSize) << static_cast<uint32_t>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(g_logger_);
  nvinfer1::INetworkDefinition* network =
      builder->createNetworkV2(explicit_batch);

  // parse onnx model
  auto parser = nvonnxparser::createParser(*network, g_logger_);
  if (!parser->parseFromFile(model_file.c_str(), verbosity)) {
    std::string msg("failed to parse onnx file");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }

  // Build the engine
  builder->setMaxBatchSize(kBatchSize);
  nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
  config->setMaxWorkspaceSize(1 << 20);
  nvinfer1::ICudaEngine* engine =
      builder->buildEngineWithConfig(*network, *config);

  *engine_ptr = engine;
  parser->destroy();
  network->destroy();
  config->destroy();
  builder->destroy();
}

void OnnxObstacleDetector::inference() {
  AINFO << "Do Inference";
}

bool OnnxObstacleDetector::Init(const std::map<std::string,
                                std::vector<int>> &shapes) {
  // create a TensorRT model from the onnx model and load it into an engine
  OnnxToTRTModel(model_file_, &engine_);
  if (engine_ == nullptr) {
    AERROR << "Fail to load obstacle ONNX model";
    return false;
  }

  // create execution context from the engine
  context_ = engine_->createExecutionContext();
  if (context_ == nullptr) {
    AERROR << "Fail to create Exceution Context";
    return false;
  }

  for (const auto& name : output_names_) {
    auto blob = std::make_shared<Blob<float>>(50, 10, 1, 1);
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

void OnnxObstacleDetector::Infer() {
  std::cout << "Infer" << std::endl;
}

BlobPtr OnnxObstacleDetector::get_blob(const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
