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
#include <NvInferVersion.h>

#include "modules/perception/common/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR != 8
class ReLUPlugin : public nvinfer1::IPlugin {
 public:
  ReLUPlugin(const ReLUParameter &param, const nvinfer1::Dims &in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      input_dims_.type[i] = in_dims.type[i];
    }
    negative_slope_ = param.negative_slope();
  }

  ReLUPlugin() {}
  ~ReLUPlugin() {}
  virtual int initialize() { return 0; }
  virtual void terminate() {}
  int getNbOutputs() const override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) override {
    nvinfer1::Dims out_dims = inputs[0];
    return out_dims;
  }

  void configure(const nvinfer1::Dims *inputDims, int nbInputs,
                 const nvinfer1::Dims *outputDims, int nbOutputs,
                 int maxBatchSize) override {
    input_dims_ = inputDims[0];
  }

  size_t getWorkspaceSize(int maxBatchSize) const override { return 0; }

  virtual int enqueue(int batchSize, const void *const *inputs, void **outputs,
                      void *workspace, cudaStream_t stream);

  size_t getSerializationSize() override { return 0; }

  void serialize(void *buffer) override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

 private:
  float negative_slope_;
  nvinfer1::Dims input_dims_;
};

#else
class ReLUPlugin : public nvinfer1::IPluginV2Ext {
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
  virtual int initialize() noexcept { return 0; }
  virtual void terminate() noexcept {}
  int32_t getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int32_t index,
                                     const nvinfer1::Dims *inputs,
                                     int32_t nbInputDims) noexcept override {
    nvinfer1::Dims out_dims = inputs[0];
    return out_dims;
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims, int32_t nbInputs,
                 const nvinfer1::Dims *outputDims, int32_t nbOutputs,
                 nvinfer1::DataType type, nvinfer1::PluginFormat format,
                 int32_t maxBatchSize) noexcept override {
    input_dims_ = inputDims[0];
  }

  size_t getWorkspaceSize(int32_t maxBatchSize)
      const noexcept override { return 0; }

  int32_t enqueue(int32_t batchSize, const void *const *inputs,
                      void *const *outputs, void *workspace,
                      cudaStream_t stream)
                      noexcept override;

  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

  nvinfer1::AsciiChar const* getPluginType()
      const noexcept override {
    return plugin_type;
  }

  nvinfer1::AsciiChar const* getPluginVersion()
      const noexcept override {
    return plugin_version;
  }

  void setPluginNamespace(const nvinfer1::AsciiChar* libNamespace)
      noexcept override {
    plugin_namespace = const_cast<nvinfer1::AsciiChar*>(libNamespace);
  }

  nvinfer1::AsciiChar const* getPluginNamespace()
      const noexcept override {
    return const_cast<nvinfer1::AsciiChar*>(plugin_namespace);
  }

  bool supportsFormat(nvinfer1::DataType type,
      nvinfer1::PluginFormat format) const noexcept override {
    return true;
  }

  void destroy() noexcept override {
    delete this;
  }

  nvinfer1::IPluginV2Ext* clone() const noexcept override {
    ReLUPlugin* p = new ReLUPlugin();
    p->negative_slope_ = negative_slope_;
    (p->input_dims_).nbDims = input_dims_.nbDims;
    for (int i = 0; i < input_dims_.nbDims; i++) {
      (p->input_dims_).d[i] = input_dims_.d[i];
    }

    p->plugin_namespace = plugin_namespace;
    return p;
  }

  bool isOutputBroadcastAcrossBatch(int32_t outputIndex,
      bool const *inputIsBroadcasted, int32_t nbInputs)
      const noexcept override {
    return false;
  }

  bool canBroadcastInputAcrossBatch(int32_t inputIndex)
      const noexcept override {
    return false;
  }

  nvinfer1::DataType getOutputDataType(int32_t index,
      nvinfer1::DataType const *inputTypes, int32_t nbInputs)
      const noexcept {
    return nvinfer1::DataType::kFLOAT;
  }

  void configurePlugin(
    nvinfer1::Dims const *inputDims, int32_t nbInputs,
    nvinfer1::Dims const *outputDims, int32_t nbOutputs,
    nvinfer1::DataType const *inputTypes, nvinfer1::DataType const *outputTypes,
    bool const *inputIsBroadcast, bool const *outputIsBroadcast,
    nvinfer1::PluginFormat floatFormat, int32_t maxBatchSize) noexcept {}

 private:
  float negative_slope_;
  nvinfer1::Dims input_dims_;

  nvinfer1::AsciiChar* plugin_namespace;
  const nvinfer1::AsciiChar* plugin_type = "";
  const nvinfer1::AsciiChar* plugin_version = "";
};

#endif
#endif

}  // namespace inference
}  // namespace perception
}  // namespace apollo
