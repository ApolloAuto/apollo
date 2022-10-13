/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include <vector>

#include "modules/perception/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

class ReLUPlugin : public nvinfer1::IPluginV2 {
 public:
  ReLUPlugin(const ReLUParameter &param, const nvinfer1::Dims &in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
    }
    negative_slope_ = param.negative_slope();
  }

  ReLUPlugin() {}
  ~ReLUPlugin() {}
  int32_t initialize() noexcept override { return 0; }
  void terminate() noexcept override {}
  int32_t getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int32_t index,
                                     const nvinfer1::Dims *inputs,
                                     int32_t nbInputDims) noexcept override {
    nvinfer1::Dims out_dims = inputs[0];
    return out_dims;
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims, int32_t nbInputs,
                           const nvinfer1::Dims *outputDims, int32_t nbOutputs,
                           nvinfer1::DataType type,
                           nvinfer1::PluginFormat format,
                           int32_t maxBatchSize) noexcept override {
    input_dims_ = inputDims[0];
  }

  size_t getWorkspaceSize(int32_t maxBatchSize) const noexcept override {
    return 0;
  }

  int32_t enqueue(int32_t batchSize, const void *const *inputs,
                  void *const *outputs, void *workspace,
                  cudaStream_t stream) noexcept override;

  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

  IPluginV2 *clone() const noexcept override {
    return const_cast<ReLUPlugin *>(this);
  }

  void destroy() noexcept override {}

  const nvinfer1::AsciiChar *getPluginNamespace() const noexcept override {
    return "apollo::perception::inference";
  }

  const nvinfer1::AsciiChar *getPluginType() const noexcept override {
    return "default";
  }

  const nvinfer1::AsciiChar *getPluginVersion() const noexcept override {
    return "1.0";
  }

  void setPluginNamespace(const nvinfer1::AsciiChar *) noexcept override {}

  bool supportsFormat(nvinfer1::DataType,
                      nvinfer1::PluginFormat) const noexcept override {
    return true;
  }

 private:
  float negative_slope_;
  nvinfer1::Dims input_dims_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
