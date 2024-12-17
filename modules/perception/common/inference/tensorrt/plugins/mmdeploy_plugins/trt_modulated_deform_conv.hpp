/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>

#include <cublas_v2.h>

#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_plugin_base.hpp"

namespace apollo {
namespace perception {
namespace inference {
class ModulatedDeformableConvPluginDynamic : public TRTPluginBase {
 public:
  ModulatedDeformableConvPluginDynamic(
      const std::string &name, const nvinfer1::Dims stride,
      const nvinfer1::Dims padding, const nvinfer1::Dims dilation,
      const int deformableGroup, const int group);

  ModulatedDeformableConvPluginDynamic(
      const std::string name, const void *data, size_t length);

  ModulatedDeformableConvPluginDynamic() = delete;

  ~ModulatedDeformableConvPluginDynamic() TRT_NOEXCEPT override;

  // IPluginV2DynamicExt Methods
  nvinfer1::IPluginV2DynamicExt *clone() const TRT_NOEXCEPT override;
  nvinfer1::DimsExprs getOutputDimensions(
      int outputIndex, const nvinfer1::DimsExprs *inputs,
      int nbInputs, nvinfer1::IExprBuilder &exprBuilder)
      TRT_NOEXCEPT override;
  bool supportsFormatCombination(
      int pos, const nvinfer1::PluginTensorDesc *ioDesc, int nbInputs,
      int nbOutputs) TRT_NOEXCEPT override;
  void configurePlugin(const nvinfer1::DynamicPluginTensorDesc *in,
                       int nbInputs,
                       const nvinfer1::DynamicPluginTensorDesc *out,
                       int nbOutputs) TRT_NOEXCEPT override;
  size_t getWorkspaceSize(const nvinfer1::PluginTensorDesc *inputs,
                          int nbInputs,
                          const nvinfer1::PluginTensorDesc *outputs,
                          int nbOutputs) const TRT_NOEXCEPT override;
  int enqueue(const nvinfer1::PluginTensorDesc *inputDesc,
              const nvinfer1::PluginTensorDesc *outputDesc,
              const void *const *inputs,
              void *const *outputs, void *workspace, cudaStream_t stream)
              TRT_NOEXCEPT override;
  void attachToContext(cudnnContext *cudnnContext, cublasContext *cublasContext,
                       nvinfer1::IGpuAllocator *gpuAllocator)
                       TRT_NOEXCEPT override;
  void detachFromContext() TRT_NOEXCEPT override;

  // IPluginV2Ext Methods
  nvinfer1::DataType getOutputDataType(
      int index, const nvinfer1::DataType *inputTypes,
      int nbInputs) const TRT_NOEXCEPT override;

  // IPluginV2 Methods
  const char *getPluginType() const TRT_NOEXCEPT override;
  const char *getPluginVersion() const TRT_NOEXCEPT override;
  int getNbOutputs() const TRT_NOEXCEPT override;
  size_t getSerializationSize() const TRT_NOEXCEPT override;
  void serialize(void *buffer) const TRT_NOEXCEPT override;

 private:
  nvinfer1::Dims mStride;
  nvinfer1::Dims mPadding;
  nvinfer1::Dims mDilation;
  int mDeformableGroup;
  int mGroup;
  bool mWithBias;

  cublasHandle_t m_cublas_handle;
};

class ModulatedDeformableConvPluginDynamicCreator :
  public TRTPluginCreatorBase {
 public:
  ModulatedDeformableConvPluginDynamicCreator();

  const char *getPluginName() const TRT_NOEXCEPT override;

  const char *getPluginVersion() const TRT_NOEXCEPT override;

  nvinfer1::IPluginV2 *createPlugin(
      const char *name, const nvinfer1::PluginFieldCollection *fc)
      TRT_NOEXCEPT override;

  nvinfer1::IPluginV2 *deserializePlugin(
      const char *name, const void *serialData,
      size_t serialLength) TRT_NOEXCEPT override;
};
}  // namespace inference
}  // namespace perception
}  // namespace apollo
