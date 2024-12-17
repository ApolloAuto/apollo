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

#include "NvInferRuntime.h"
#include "NvInferVersion.h"
#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_plugin_helper.hpp"

#include <vector>
#include <string>

namespace apollo {
namespace perception {
namespace inference {

#if NV_TENSORRT_MAJOR > 7
#define TRT_NOEXCEPT noexcept
#else
#define TRT_NOEXCEPT
#endif

class TRTPluginBase : public nvinfer1::IPluginV2DynamicExt {
 public:
  explicit TRTPluginBase(const std::string &name) : mLayerName(name) {}
  // IPluginV2 Methods
  const char *getPluginVersion() const TRT_NOEXCEPT override { return "1"; }
  int initialize() TRT_NOEXCEPT override { return STATUS_SUCCESS; }
  void terminate() TRT_NOEXCEPT override {}
  void destroy() TRT_NOEXCEPT override { delete this; }
  void setPluginNamespace(const char *pluginNamespace) TRT_NOEXCEPT override {
    mNamespace = pluginNamespace;
  }
  const char *getPluginNamespace() const TRT_NOEXCEPT override {
    return mNamespace.c_str();
  }

  void configurePlugin(const nvinfer1::DynamicPluginTensorDesc *in,
                       int nbInputs,
                       const nvinfer1::DynamicPluginTensorDesc *out,
                       int nbOutputs) TRT_NOEXCEPT override {}

  size_t getWorkspaceSize(const nvinfer1::PluginTensorDesc *inputs,
                          int nbInputs,
                          const nvinfer1::PluginTensorDesc *outputs,
                          int nbOutputs) const TRT_NOEXCEPT override {
    return 0;
  }

  void attachToContext(
    cudnnContext *cudnnContext, cublasContext *cublasContext,
    nvinfer1::IGpuAllocator *gpuAllocator) TRT_NOEXCEPT override {}

  void detachFromContext() TRT_NOEXCEPT override {}

 protected:
  const std::string mLayerName;
  std::string mNamespace;

#if NV_TENSORRT_MAJOR < 8

 protected:
  // To prevent compiler warnings.
  using nvinfer1::IPluginV2DynamicExt::canBroadcastInputAcrossBatch;
  using nvinfer1::IPluginV2DynamicExt::enqueue;
  using nvinfer1::IPluginV2DynamicExt::getOutputDimensions;
  using nvinfer1::IPluginV2DynamicExt::isOutputBroadcastAcrossBatch;
  using nvinfer1::IPluginV2DynamicExt::supportsFormat;
#endif
};

class TRTPluginCreatorBase : public nvinfer1::IPluginCreator {
 public:
  const char *getPluginVersion() const TRT_NOEXCEPT override { return "1"; };

  const nvinfer1::PluginFieldCollection *getFieldNames() TRT_NOEXCEPT override {
    return &mFC;
  }

  void setPluginNamespace(const char *pluginNamespace) TRT_NOEXCEPT override {
    mNamespace = pluginNamespace;
  }

  const char *getPluginNamespace() const TRT_NOEXCEPT override {
    return mNamespace.c_str();
  }

 protected:
  nvinfer1::PluginFieldCollection mFC;
  std::vector<nvinfer1::PluginField> mPluginAttributes;
  std::string mNamespace;
};
}  // namespace inference
}  // namespace perception
}  // namespace apollo
