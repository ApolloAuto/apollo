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

#include <NvInferVersion.h>

#include <string>

#include "modules/perception/common/inference/tensorrt/rt_common.h"
namespace apollo {
namespace perception {
namespace inference {

#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR != 8
class SoftmaxPlugin : public nvinfer1::IPlugin {
 public:
  SoftmaxPlugin(const SoftmaxParameter &param, nvinfer1::Dims in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      input_dims_.type[i] = in_dims.type[i];
    }
    axis_ = param.axis() - 1;
    CHECK_GE(axis_, 0);
    CHECK_LE(axis_ + 1, input_dims_.nbDims);

    inner_num_ = 1;
    for (int i = axis_ + 1; i < input_dims_.nbDims; i++) {
      inner_num_ *= input_dims_.d[i];
    }
    outer_num_ = 1;
    for (int i = 0; i < axis_; i++) {
      outer_num_ *= input_dims_.d[i];
    }
    cudnnCreateTensorDescriptor(&input_desc_);
    cudnnCreateTensorDescriptor(&output_desc_);
  }

  SoftmaxPlugin() {}

  ~SoftmaxPlugin() {
    cudnnDestroyTensorDescriptor(input_desc_);
    cudnnDestroyTensorDescriptor(output_desc_);
  }
  virtual int initialize() {
    cudnnCreate(&cudnn_);  // initialize cudnn and cublas
    cublasCreate(&cublas_);
    return 0;
  }
  virtual void terminate() {
    cublasDestroy(cublas_);
    cudnnDestroy(cudnn_);
  }
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

  int enqueue(int batchSize, const void *const *inputs, void **outputs,
              void *workspace, cudaStream_t stream) override;

  size_t getSerializationSize() override { return 0; }

  void serialize(void *buffer) override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

 private:
  cudnnHandle_t cudnn_;
  cublasHandle_t cublas_;
  nvinfer1::Dims input_dims_;
  int axis_;
  int inner_num_;
  int outer_num_;
  cudnnTensorDescriptor_t input_desc_;
  cudnnTensorDescriptor_t output_desc_;
};

#else
class SoftmaxPlugin : public nvinfer1::IPluginV2 {
 public:
  SoftmaxPlugin(const char *layer_name, const void *serial_data,
                size_t serial_length) {
    const char *p = reinterpret_cast<const char *>(serial_data);
    // CHECK_EQ(getPluginType(), Name());
    // p += (Name().size() + strlen(name_));

    input_dims_.nbDims = reinterpret_cast<const int *>(p)[0];
    p += sizeof(int);
    for (int i = 0; i < input_dims_.nbDims; ++i) {
      input_dims_.d[i] = reinterpret_cast<const int *>(p)[0];
      p += sizeof(int);
      // input_dims_.type[i] = static_cast<nvinfer1::DimensionType>(
      //     reinterpret_cast<const int *>(p)[0]);
      // p += sizeof(int);
    }

    inner_num_ = reinterpret_cast<const int *>(p)[0];
    p += sizeof(int);

    outer_num_ = reinterpret_cast<const int *>(p)[0];
    p += sizeof(int);

    axis_ = reinterpret_cast<const int *>(p)[0];
    CHECK_GT(axis_, 0);
    // CHECK_LE(axis_ + 1, input_dims_.nbDims);
    CHECK_LE(axis_, input_dims_.nbDims);
    p += sizeof(int);

    CHECK_EQ(p, reinterpret_cast<const char *>(serial_data) + serial_length);
    CHECK_EQ(getSerializationSize(), serial_length);

    cudnnCreateTensorDescriptor(&input_desc_);
    cudnnCreateTensorDescriptor(&output_desc_);
  }

  SoftmaxPlugin(const SoftmaxParameter &param, nvinfer1::Dims in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      // input_dims_.type[i] = in_dims.type[i];
    }
    axis_ = param.axis() - 1;
    CHECK_GE(axis_, 0);
    CHECK_LE(axis_ + 1, input_dims_.nbDims);

    inner_num_ = 1;
    for (int i = axis_ + 1; i < input_dims_.nbDims; i++) {
      inner_num_ *= input_dims_.d[i];
    }
    outer_num_ = 1;
    for (int i = 0; i < axis_; i++) {
      outer_num_ *= input_dims_.d[i];
    }
    cudnnCreateTensorDescriptor(&input_desc_);
    cudnnCreateTensorDescriptor(&output_desc_);
  }

  SoftmaxPlugin(int axis, nvinfer1::Dims in_dims) {
    input_dims_.nbDims = in_dims.nbDims;
    for (int i = 0; i < in_dims.nbDims; i++) {
      input_dims_.d[i] = in_dims.d[i];
      // input_dims_.type[i] = in_dims.type[i];
    }
    axis_ = axis - 1;
    CHECK_GE(axis_, 0);
    CHECK_LE(axis_ + 1, input_dims_.nbDims);

    inner_num_ = 1;
    for (int i = axis_ + 1; i < input_dims_.nbDims; i++) {
      inner_num_ *= input_dims_.d[i];
    }
    outer_num_ = 1;
    for (int i = 0; i < axis_; i++) {
      outer_num_ *= input_dims_.d[i];
    }
    cudnnCreateTensorDescriptor(&input_desc_);
    cudnnCreateTensorDescriptor(&output_desc_);
  }
  SoftmaxPlugin() {}
  ~SoftmaxPlugin() {
    cudnnDestroyTensorDescriptor(input_desc_);
    cudnnDestroyTensorDescriptor(output_desc_);
  }

  virtual int initialize() noexcept {
    cudnnCreate(&cudnn_);  // initialize cudnn and cublas
    cublasCreate(&cublas_);
    return 0;
  }
  virtual void terminate() noexcept {
    cublasDestroy(cublas_);
    cudnnDestroy(cudnn_);
  }
  int getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) noexcept override {
    nvinfer1::Dims out_dims = inputs[0];
    return out_dims;
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims, int nbInputs,
                           const nvinfer1::Dims *outputDims, int nbOutputs,
                           nvinfer1::DataType type,
                           nvinfer1::PluginFormat format,
                           int maxBatchSize) noexcept override {
    input_dims_ = inputDims[0];
  }
  size_t getWorkspaceSize(int maxBatchSize) const noexcept override {
    return 0;
  }

  int enqueue(int batchSize, const void *const *inputs, void *const *outputs,
              void *workspace, cudaStream_t stream) noexcept override;

  size_t getSerializationSize() const noexcept override {
    /* return sizeof(size_t) + Name().size() + \ */
    return sizeof(int) + sizeof(int) * 1 * input_dims_.nbDims +
           sizeof(inner_num_) + sizeof(outer_num_) + sizeof(axis_);
  }

  void serialize(void *buffer) const noexcept override {
    char *p = reinterpret_cast<char *>(buffer);
    // setPluginType(&p, Name());

    reinterpret_cast<int *>(p)[0] = input_dims_.nbDims;
    p += sizeof(int);
    for (int i = 0; i < input_dims_.nbDims; ++i) {
      reinterpret_cast<int *>(p)[0] = input_dims_.d[i];
      p += sizeof(int);
      // reinterpret_cast<int *>(p)[0] = static_cast<int>(input_dims_.type[i]);
      // p += sizeof(int);
    }

    reinterpret_cast<int *>(p)[0] = inner_num_;
    p += sizeof(int);

    reinterpret_cast<int *>(p)[0] = outer_num_;
    p += sizeof(int);

    reinterpret_cast<int *>(p)[0] = axis_;
    p += sizeof(int);
  }

  std::string Name() const { return "softmax_plugin"; }

  const char *getPluginType() const noexcept override {
    return "softmax_plugin";
  };

  const char *getPluginVersion() const noexcept override { return "1"; }

  void setPluginNamespace(const char *s_name) noexcept override {
    name_ = s_name;
  }

  const char *getPluginNamespace() const noexcept override { return "camera"; }
  bool supportsFormat(nvinfer1::DataType type,
                      nvinfer1::PluginFormat format) const noexcept override {
    return true;
  }

  void destroy() noexcept override { delete this; }

  nvinfer1::IPluginV2 *clone() const noexcept override {
    SoftmaxPlugin *p = new SoftmaxPlugin();
    cudnnCreate(&(p->cudnn_));  // initialize cudnn and cublas
    cublasCreate(&(p->cublas_));
    p->axis_ = axis_;
    p->inner_num_ = inner_num_;
    p->outer_num_ = outer_num_;
    (p->input_dims_).nbDims = input_dims_.nbDims;
    for (int i = 0; i < input_dims_.nbDims; i++) {
      (p->input_dims_).d[i] = input_dims_.d[i];
    }
    cudnnCreateTensorDescriptor(&(p->input_desc_));
    cudnnCreateTensorDescriptor(&(p->output_desc_));
    return p;
  }

  nvinfer1::DataType getOutputDataType(int32_t index,
                                       nvinfer1::DataType const *inputTypes,
                                       int32_t nbInputs) const noexcept {
    return nvinfer1::DataType::kFLOAT;
  }

  void configurePlugin(nvinfer1::Dims const *inputDims, int32_t nbInputs,
                       nvinfer1::Dims const *outputDims, int32_t nbOutputs,
                       nvinfer1::DataType const *inputTypes,
                       nvinfer1::DataType const *outputTypes,
                       bool const *inputIsBroadcast,
                       bool const *outputIsBroadcast,
                       nvinfer1::PluginFormat floatFormat,
                       int32_t maxBatchSize) noexcept {}

 private:
  const char *name_ = "camera";
  cudnnHandle_t cudnn_;
  cublasHandle_t cublas_;
  nvinfer1::Dims input_dims_;
  int axis_;
  int inner_num_;
  int outer_num_;
  cudnnTensorDescriptor_t input_desc_;
  cudnnTensorDescriptor_t output_desc_;
};

class SoftmaxPluginCreator : public nvinfer1::IPluginCreator {
 public:
  SoftmaxPluginCreator() {}
  ~SoftmaxPluginCreator() {
    // terminae();
  }
  nvinfer1::IPluginV2 *createPlugin(
      const char *name,
      const nvinfer1::PluginFieldCollection *fc) noexcept override {
    int axis;
    nvinfer1::Dims in_dims;
    const nvinfer1::PluginField *fields = fc->fields;
    for (int i = 0; i < fc->nbFields; ++i) {
      const char *attrName = fields[i].name;
      if (!strcmp(attrName, "axis")) {
        // ASSERT(fields[i].type == nvinfer1::PluginFieldType::kINT32);
        axis = *(static_cast<const int32_t *>(fields[i].data));
      } else if (!strcmp(attrName, "in_dims")) {
        // ASSERT(fields[i].type ==nvinfer1::PluginFieldType::kDIMS);
        in_dims = *(static_cast<const nvinfer1::Dims *>(fields[i].data));
      }
    }

    SoftmaxPlugin *obj = new SoftmaxPlugin(axis, in_dims);
    obj->setPluginNamespace(mNamespaces.c_str());
    return obj;
  }
  nvinfer1::IPluginV2 *deserializePlugin(
      const char *name, const void *serial_data,
      size_t serial_length) noexcept override {
    return new SoftmaxPlugin(name, serial_data, serial_length);
  }
  const char *getPluginName() const noexcept override {
    return "softmax_plugincamera";
  }
  const char *getPluginVersion() const noexcept override { return "1"; }
  const char *getPluginNamespace() const noexcept override { return "camera"; }
  void setPluginNamespace(const char *s_name) noexcept override {
    mNamespaces = s_name;
  }
  const nvinfer1::PluginFieldCollection *getFieldNames() noexcept override {
    return nullptr;
  }

 private:
  std::string mNamespaces = "camera";
};
REGISTER_TENSORRT_PLUGIN(SoftmaxPluginCreator);
#endif
#endif

}  // namespace inference
}  // namespace perception
}  // namespace apollo
