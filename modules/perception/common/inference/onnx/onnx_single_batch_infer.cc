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
#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR == 8
#include "modules/perception/common/inference/tensorrt/rt_legacy.h"
#endif
#endif

#include <sys/stat.h>

#include <fstream>

#include <cuda_runtime_api.h>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/perception/common/inference/onnx/onnx_single_batch_infer.h"

#if GPU_PLATFORM == NVIDIA
// Logger for TensorRT info/warning/errors
class OnnxLogger : public nvinfer1::ILogger {
  void log(Severity severity, const char* msg) noexcept override {
    switch (severity) {
      case Severity::kINTERNAL_ERROR:
      case Severity::kERROR:
        AERROR << msg;
        break;
      case Severity::kWARNING:
        AWARN << msg;
        break;
      case Severity::kINFO:
      case Severity::kVERBOSE:
        ADEBUG << msg;
        break;
      default:
        break;
    }
  }
} onnx_gLogger;
#elif GPU_PLATFORM == AMD
class OnnxLogger : {
} onnx_gLogger;
namespace nvinfer1 {
class ICudaEngine {};
class IExecutionContext {};
}  // namespace nvinfer1
#endif

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

base::BlobPtr<float> SingleBatchInference::get_blob(const std::string& name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

SingleBatchInference::SingleBatchInference(
    const std::string& model_file, const float score_threshold,
    const std::vector<std::string>& outputs,
    const std::vector<std::string>& inputs)
    : model_file_(model_file),
      score_threshold_(score_threshold),
      output_names_(outputs),
      input_names_(inputs) {}

SingleBatchInference::SingleBatchInference(
    const std::string& model_file, const std::vector<std::string>& outputs,
    const std::vector<std::string>& inputs)
    : model_file_(model_file), output_names_(outputs), input_names_(inputs) {}

SingleBatchInference::~SingleBatchInference() {}

bool SingleBatchInference::LoadCache(const std::string& path) {
  struct stat buffer;
  if (stat(path.c_str(), &buffer) != 0) {
    AERROR << "[INFO] cannot find model cache : " << path
           << ", it will take minutes to generate...";
    return false;
  }
  return true;
}

bool SingleBatchInference::Init(
    const std::map<std::string, std::vector<int>>& shapes) {
  BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
  BASE_GPU_CHECK(cudaStreamCreate(&stream_));

  // create a TensorRT model from the onnx model and load it into an engine
  OnnxToTRTModel(model_file_);

  if (engine_ == nullptr) {
    AERROR << "Fail to load obstacle ONNX model";
    return false;
  }

  // create execution context from the engine
#if GPU_PLATFORM == NVIDIA
  context_ = engine_->createExecutionContext();
  if (context_ == nullptr) {
    AERROR << "Fail to create Exceution Context";
    return false;
  }

  for (const auto& name : output_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      auto blob = std::make_shared<Blob<float>>(iter->second);
      blobs_.emplace(name, blob);
    }
  }

  for (const auto& name : input_names_) {
    auto iter = shapes.find(name);
    if (iter != shapes.end()) {
      auto blob = std::make_shared<Blob<float>>(iter->second);
      blobs_.emplace(name, blob);
    }
  }
  buffers_.resize(input_names_.size() + output_names_.size());

  init_blob(&input_names_);
  init_blob(&output_names_);

#elif GPU_PLATFORM == AMD
  assert(0 && "OnnxObstacleDetector::Init() is not implemented yet");
#endif
  return true;
}

bool SingleBatchInference::OnnxToTRTModel(const std::string& model_file) {
#if GPU_PLATFORM == NVIDIA
  int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kVERBOSE);
  kBatchSize = 1;

  if (gpu_id_ < 0) {
    AERROR << "must use gpu mode";
    return false;
  }
  // create the builder
  nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(onnx_gLogger);
  nvinfer1::INetworkDefinition* network = builder->createNetworkV2(
      1U << static_cast<int>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));

  // parse onnx model
  nvonnxparser::IParser* parser =
      nvonnxparser::createParser(*network, onnx_gLogger);

  const char* onnx_filename = model_file.c_str();
  parser->parseFromFile(onnx_filename, verbosity);

  // Build the engine
  builder->setMaxBatchSize(kBatchSize);
  nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
  bool use_fp16 = builder->platformHasFastFp16();
  if (use_fp16) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }
  config->setMaxWorkspaceSize(1 << 30);

  auto trt_cache_path = model_file + ".trt.engine";

  if (!LoadCache(trt_cache_path)) {
    engine_ = builder->buildEngineWithConfig(*network, *config);

    if (engine_->serialize()) {
      nvinfer1::IHostMemory* serialized_model(engine_->serialize());
      std::ofstream f(trt_cache_path, std::ios::binary);

      f.write(reinterpret_cast<char*>(serialized_model->data()),
              serialized_model->size());
      serialized_model->destroy();
      AERROR << "Saving serialized model file to " << trt_cache_path;
    }
  } else {
    AINFO << "Loading TensorRT engine from serialized model file...";
    std::ifstream planFile(trt_cache_path);

    if (!planFile.is_open()) {
      AERROR << "Could not open serialized model";
      return false;
    }
    initLibNvInferPlugins(&onnx_gLogger, "");

    AINFO << "success open serialized model";
    std::stringstream planBuffer;
    planBuffer << planFile.rdbuf();
    std::string plan = planBuffer.str();
    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(onnx_gLogger);

    engine_ = runtime->deserializeCudaEngine(
        static_cast<const void*>(plan.data()), plan.size(), nullptr);
    runtime->destroy();
  }
  return true;
#elif GPU_PLATFORM == AMD
  assert(0 && "OnnxObstacleDetector::OnnxToTRTModel() is not implemented yet");
#endif
}

void SingleBatchInference::init_blob(std::vector<std::string>* names) {
  for (auto name : *names) {
    auto blob = get_blob(name);

    std::vector<int> shape(blob->shape().begin(), blob->shape().end());

    buffers_[bindingIndex_] = blob->mutable_gpu_data();
    bindingIndex_ += 1;
  }
}

void SingleBatchInference::Infer() {
  BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
  BASE_GPU_CHECK(cudaStreamSynchronize(stream_));

  for (auto name : input_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      blob->gpu_data();
    }
  }
  // If `out_blob->mutable_cpu_data()` is invoked outside,
  // HEAD will be set to CPU, and `out_blob->mutable_gpu_data()`
  // after `enqueue` will copy data from CPU to GPU,
  // which will overwrite the `inference` results.
  // `out_blob->gpu_data()` will set HEAD to SYNCED,
  // then no copy happends after `enqueue`.
  for (auto name : output_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      blob->gpu_data();
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    context_->enqueue(max_batch_size_, &buffers_[0], stream_, nullptr);
  }

  for (auto name : output_names_) {
    auto blob = get_blob(name);
    if (blob != nullptr) {
      blob->mutable_gpu_data();
    }
  }
}

}  // namespace inference
}  // namespace perception
}  // namespace apollo
