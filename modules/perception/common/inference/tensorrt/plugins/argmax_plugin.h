/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include <limits>
#include <NvInferVersion.h>

#include "modules/perception/common/proto/rt.pb.h"

#include "modules/perception/common/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR != 8
class ArgMax1Plugin : public nvinfer1::IPlugin {
 public:
  ArgMax1Plugin(const ArgMaxParameter &argmax_param, nvinfer1::Dims in_dims)
      : float_min_(std::numeric_limits<float>::min()) {
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      input_dims_.type[i] = in_dims.type[i];
    }
    axis_ = argmax_param.axis();
    out_max_val_ = argmax_param.out_max_val();
    top_k_ = argmax_param.top_k();
    CHECK_GE(top_k_, static_cast<size_t>(1))
        << "top k must not be less than 1.";
    output_dims_ = input_dims_;
    output_dims_.d[0] = 1;
    if (out_max_val_) {
      // Produces max_ind and max_val
      output_dims_.d[0] = 2;
    }
  }

  /**
   * \brief get the number of outputs from the layer
   *
   * \return the number of outputs
   *
   * this function is called by the implementations of INetworkDefinition and
   * IBuilder. In particular, it is called prior to any call to initialize().
   */
  virtual int initialize() { return 0; }
  virtual void terminate() {}
  int getNbOutputs() const override { return 1; }
  virtual nvinfer1::Dims getOutputDimensions(int index,
                                             const nvinfer1::Dims *inputs,
                                             int nbInputDims) {
    input_dims_ = inputs[0];
    for (int i = 1; i < input_dims_.nbDims; i++) {
      output_dims_.d[i] = input_dims_.d[i];
    }
    return output_dims_;
  }

  void configure(const nvinfer1::Dims *inputDims, int nbInputs,
                 const nvinfer1::Dims *outputDims, int nbOutputs,
                 int maxBatchSize) override {
    input_dims_ = inputDims[0];
    for (int i = 1; i < input_dims_.nbDims; i++) {
      output_dims_.d[i] = input_dims_.d[i];
    }
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

  virtual ~ArgMax1Plugin() {}

 private:
  bool out_max_val_;
  size_t top_k_;
  int axis_;
  float float_min_;
  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;
};

#else
class ArgMax1Plugin : public nvinfer1::IPluginV2Ext {
 public:
  ArgMax1Plugin(const ArgMaxParameter &argmax_param, nvinfer1::Dims in_dims)
      : float_min_(std::numeric_limits<float>::min()) {
    input_dims_.nbDims = in_dims.nbDims;
    CHECK_GT(input_dims_.nbDims, 0);
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
    }
    axis_ = argmax_param.axis();
    out_max_val_ = argmax_param.out_max_val();
    top_k_ = argmax_param.top_k();
    CHECK_GE(top_k_, static_cast<size_t>(1))
        << "top k must not be less than 1.";
    output_dims_ = input_dims_;
    output_dims_.d[0] = 1;
    if (out_max_val_) {
      // Produces max_ind and max_val
      output_dims_.d[0] = 2;
    }
  }

  /**
   * \brief get the number of outputs from the layer
   *
   * \return the number of outputs
   *
   * this function is called by the implementations of INetworkDefinition and
   * IBuilder. In particular, it is called prior to any call to initialize().
   */
  virtual int initialize() noexcept { return 0; }
  virtual void terminate() noexcept {}
  int32_t getNbOutputs() const noexcept override { return 1; }
  nvinfer1::Dims getOutputDimensions(int32_t index,
        const nvinfer1::Dims *inputs, int32_t nbInputDims) noexcept override {
    input_dims_ = inputs[0];
    for (int i = 1; i < input_dims_.nbDims; i++) {
      output_dims_.d[i] = input_dims_.d[i];
    }
    return output_dims_;
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims, int32_t nbInputs,
                 const nvinfer1::Dims *outputDims, int32_t nbOutputs,
                 nvinfer1::DataType type, nvinfer1::PluginFormat format,
                 int32_t maxBatchSize) noexcept override {
    input_dims_ = inputDims[0];
    for (int i = 1; i < input_dims_.nbDims; i++) {
      output_dims_.d[i] = input_dims_.d[i];
    }
  }

  size_t getWorkspaceSize(int32_t maxBatchSize)
    const noexcept override { return 0; }

  int32_t enqueue(int32_t batchSize, const void *const *inputs,
            void *const *outputs, void *workspace,
            cudaStream_t stream) noexcept override;

  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

  ArgMax1Plugin() {}
  virtual ~ArgMax1Plugin() {}

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
    ArgMax1Plugin* p = new ArgMax1Plugin();
    p->out_max_val_ = out_max_val_;
    p->top_k_ = top_k_;
    p->axis_ = axis_;
    p->float_min_ = float_min_;

    (p->input_dims_).nbDims = input_dims_.nbDims;
    for (int i = 0; i < input_dims_.nbDims; i++) {
      (p->input_dims_).d[i] = input_dims_.d[i];
    }
    p->output_dims_ = p->input_dims_;
    p->output_dims_.d[0] = 1;
    if (p->out_max_val_) {
      // Produces max_ind and max_val
      p->output_dims_.d[0] = 2;
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
      nvinfer1::DataType const *inputTypes,
      int32_t nbInputs) const noexcept {
    return nvinfer1::DataType::kFLOAT;
  }

  void configurePlugin(
    nvinfer1::Dims const *inputDims, int32_t nbInputs,
    nvinfer1::Dims const *outputDims, int32_t nbOutputs,
    nvinfer1::DataType const *inputTypes, nvinfer1::DataType const *outputTypes,
    bool const *inputIsBroadcast, bool const *outputIsBroadcast,
    nvinfer1::PluginFormat floatFormat, int32_t maxBatchSize) noexcept {}

 private:
  bool out_max_val_;
  size_t top_k_;
  int axis_;
  float float_min_;
  nvinfer1::Dims input_dims_;
  nvinfer1::Dims output_dims_;

  nvinfer1::AsciiChar* plugin_namespace;
  const nvinfer1::AsciiChar* plugin_type = "";
  const nvinfer1::AsciiChar* plugin_version = "";
};

#endif
#endif

}  // namespace inference
}  // namespace perception
}  // namespace apollo
