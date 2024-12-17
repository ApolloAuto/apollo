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

#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_modulated_deform_conv.hpp"

#include <assert.h>

#include <chrono>
#include <algorithm>

#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_modulated_deform_conv_kernel.hpp"
#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_serialize.hpp"

namespace apollo {
namespace perception {
namespace inference {

namespace {
static const char *PLUGIN_VERSION{"1"};
static const char *PLUGIN_NAME{"MMCVModulatedDeformConv2d"};
}  // namespace

ModulatedDeformableConvPluginDynamic::ModulatedDeformableConvPluginDynamic(
    const std::string &name, const nvinfer1::Dims stride,
    const nvinfer1::Dims padding, const nvinfer1::Dims dilation,
    const int deformableGroup, const int group)
    : TRTPluginBase(name),
      mStride(stride),
      mPadding(padding),
      mDilation(dilation),
      mDeformableGroup(deformableGroup),
      mGroup(group) {
  mWithBias = false;
}

ModulatedDeformableConvPluginDynamic::ModulatedDeformableConvPluginDynamic(
  const std::string name,
  const void *data,
  size_t length)
  : TRTPluginBase(name) {
    deserialize_value(&data, &length, &mStride);
    deserialize_value(&data, &length, &mPadding);
    deserialize_value(&data, &length, &mDilation);
    deserialize_value(&data, &length, &mDeformableGroup);
    deserialize_value(&data, &length, &mGroup);
    mWithBias = false;
}
ModulatedDeformableConvPluginDynamic::~ModulatedDeformableConvPluginDynamic() {}

nvinfer1::IPluginV2DynamicExt *ModulatedDeformableConvPluginDynamic::clone()
const TRT_NOEXCEPT {
  ModulatedDeformableConvPluginDynamic *plugin =
    new ModulatedDeformableConvPluginDynamic(
      mLayerName, mStride, mPadding, mDilation, mDeformableGroup, mGroup);
  plugin->setPluginNamespace(getPluginNamespace());

  return plugin;
}

static const nvinfer1::IDimensionExpr *get_hw(
  const nvinfer1::IDimensionExpr *input,
  const nvinfer1::IDimensionExpr *weight,
  const nvinfer1::IDimensionExpr *stride,
  const nvinfer1::IDimensionExpr *pad,
  const nvinfer1::IDimensionExpr *dilation,
  nvinfer1::IExprBuilder &exprBuilder) {
  using DimOp = nvinfer1::DimensionOperation;
  auto expr_1 = exprBuilder.constant(1);

  // d*(w-1)+1
  auto kernel_0 = exprBuilder.operation(DimOp::kSUB, *weight, *expr_1);
  auto kernel_1 = exprBuilder.operation(DimOp::kPROD, *dilation, *kernel_0);
  auto kernel = exprBuilder.operation(DimOp::kSUM, *kernel_1, *expr_1);

  // (1+2*p-k)//stride -1
  auto out_0 = exprBuilder.operation(DimOp::kSUM, *pad, *pad);
  auto out_1 = exprBuilder.operation(DimOp::kSUM, *input, *out_0);
  auto out_2 = exprBuilder.operation(DimOp::kSUB, *out_1, *kernel);
  auto out_3 = exprBuilder.operation(DimOp::kFLOOR_DIV, *out_2, *stride);
  auto out = exprBuilder.operation(DimOp::kSUM, *out_3, *expr_1);

  return out;
}

nvinfer1::DimsExprs ModulatedDeformableConvPluginDynamic::getOutputDimensions(
    int outputIndex, const nvinfer1::DimsExprs *inputs, int nbInputs,
    nvinfer1::IExprBuilder &exprBuilder) TRT_NOEXCEPT {
  using DimOp = nvinfer1::DimensionOperation;
  auto weight_dim = inputs[3].d;
  nvinfer1::DimsExprs ret;
  ret.nbDims = 4;
  ret.d[0] = inputs[0].d[0];
  ret.d[1] = inputs[3].d[0];

  auto input_h = inputs[0].d[2];
  auto input_w = inputs[0].d[3];
  auto weight_h = weight_dim[2];
  auto weight_w = weight_dim[3];
  auto dilation_w = exprBuilder.constant(mDilation.d[0]);
  auto dilation_h = exprBuilder.constant(mDilation.d[1]);
  auto pad_w = exprBuilder.constant(mPadding.d[0]);
  auto pad_h = exprBuilder.constant(mPadding.d[1]);
  auto stride_w = exprBuilder.constant(mStride.d[0]);
  auto stride_h = exprBuilder.constant(mStride.d[1]);
  auto expr_1 = exprBuilder.constant(1);
  auto expr_2 = exprBuilder.constant(2);

  ret.d[2] = get_hw(input_h, weight_h, stride_h,
                    pad_h, dilation_h, exprBuilder);
  ret.d[3] = get_hw(input_w, weight_w, stride_w, pad_w,
                    dilation_w, exprBuilder);

  return ret;
}

bool ModulatedDeformableConvPluginDynamic::supportsFormatCombination(
    int pos, const nvinfer1::PluginTensorDesc *ioDesc,
    int nbInputs, int nbOutputs) TRT_NOEXCEPT {
  if (pos == 0) {
    return ((ioDesc[pos].type == nvinfer1::DataType::kFLOAT ||
             ioDesc[pos].type == nvinfer1::DataType::kHALF) &&
            ioDesc[pos].format == nvinfer1::TensorFormat::kLINEAR);
  } else {
    return ioDesc[pos].type == ioDesc[0].type &&
           ioDesc[pos].format == ioDesc[0].format;
  }
}

void ModulatedDeformableConvPluginDynamic::configurePlugin(
    const nvinfer1::DynamicPluginTensorDesc *inputs, int nbInputs,
    const nvinfer1::DynamicPluginTensorDesc *outputs, int nbOutputs)
    TRT_NOEXCEPT {
  if (nbInputs == 5) {
    mWithBias = true;
  }
}

size_t ModulatedDeformableConvPluginDynamic::getWorkspaceSize(
    const nvinfer1::PluginTensorDesc *inputs, int nbInputs,
    const nvinfer1::PluginTensorDesc *outputs, int nbOutputs)
    const TRT_NOEXCEPT {
  int sizeof_dtype = getElementSize(outputs[0].type);

  int batch_size = inputs[0].dims.d[0];
  int nInputPlane = inputs[0].dims.d[1];
  int inputHeight = inputs[0].dims.d[2];
  int inputWidth = inputs[0].dims.d[3];

  int nOutputPlane = outputs[0].dims.d[1];
  int outputHeight = outputs[0].dims.d[2];
  int outputWidth = outputs[0].dims.d[3];

  int kW = inputs[3].dims.d[2];
  int kH = inputs[3].dims.d[3];
  int im2col_step = std::min(32, batch_size);

  size_t col_size =
      getAlignedSize(nInputPlane * kW * kH *
                     outputHeight * outputWidth * sizeof_dtype);

  return col_size;
}

int ModulatedDeformableConvPluginDynamic::enqueue(
  const nvinfer1::PluginTensorDesc *inputDesc,
  const nvinfer1::PluginTensorDesc *outputDesc,
  const void *const *inputs, void *const *outputs,
  void *workSpace,
  cudaStream_t stream) TRT_NOEXCEPT {
  int batch = inputDesc[0].dims.d[0];
  int channels = inputDesc[0].dims.d[1];
  int height = inputDesc[0].dims.d[2];
  int width = inputDesc[0].dims.d[3];
  int channels_out = outputDesc[0].dims.d[1];
  int kernel_h = inputDesc[3].dims.d[2];
  int kernel_w = inputDesc[3].dims.d[3];

  const void *x = inputs[0];
  const void *offset = inputs[1];
  const void *mask = inputs[2];
  const void *weight = inputs[3];
  const void *bias = mWithBias ? inputs[4] : nullptr;
  void *output = outputs[0];
  int im2col_step = std::min(batch, 32);

  // TODO(yanjiaxing): add fp16 support
  auto data_type = inputDesc[0].type;
  switch (data_type) {
    case nvinfer1::DataType::kFLOAT:
      ModulatedDeformConvForwardCUDAKernelLauncher<float>(
          reinterpret_cast<const float*>(x),
          reinterpret_cast<const float*>(weight),
          reinterpret_cast<const float*>(bias),
          reinterpret_cast<const float*>(offset),
          reinterpret_cast<const float*>(mask),
          reinterpret_cast<float*>(output),
          workSpace, batch, channels, height, width, channels_out,
          kernel_w, kernel_h, mStride.d[0], mStride.d[1], mPadding.d[0],
          mPadding.d[1], mDilation.d[0], mDilation.d[1], mGroup,
          mDeformableGroup, im2col_step, m_cublas_handle, stream);
      break;
    case nvinfer1::DataType::kHALF:
      ModulatedDeformConvForwardCUDAKernelLauncher<half>(
          reinterpret_cast<const half*>(x),
          reinterpret_cast<const half*>(weight),
          reinterpret_cast<const half*>(bias),
          reinterpret_cast<const half*>(offset),
          reinterpret_cast<const half*>(mask),
          reinterpret_cast<half*>(output),
          workSpace, batch, channels, height, width, channels_out,
          kernel_w, kernel_h, mStride.d[0], mStride.d[1], mPadding.d[0],
          mPadding.d[1], mDilation.d[0], mDilation.d[1], mGroup,
          mDeformableGroup, im2col_step, m_cublas_handle, stream);
      break;
    default:
      return 1;
      break;
  }

  return 0;
}

nvinfer1::DataType ModulatedDeformableConvPluginDynamic::getOutputDataType(
    int index, const nvinfer1::DataType *inputTypes, int nbInputs)
    const TRT_NOEXCEPT {
  return inputTypes[0];
}

// IPluginV2 Methods
const char *ModulatedDeformableConvPluginDynamic::getPluginType()
  const TRT_NOEXCEPT {
  return PLUGIN_NAME;
}

const char *ModulatedDeformableConvPluginDynamic::getPluginVersion()
  const TRT_NOEXCEPT {
  return PLUGIN_VERSION;
}

int ModulatedDeformableConvPluginDynamic::getNbOutputs()
  const TRT_NOEXCEPT { return 1; }

size_t ModulatedDeformableConvPluginDynamic::getSerializationSize()
  const TRT_NOEXCEPT {
  return serialized_size(mStride) + serialized_size(mPadding) +
         serialized_size(mDilation) + serialized_size(mDeformableGroup) +
         serialized_size(mGroup);
}

void ModulatedDeformableConvPluginDynamic::serialize(void *buffer)
  const TRT_NOEXCEPT {
  serialize_value(&buffer, mStride);
  serialize_value(&buffer, mPadding);
  serialize_value(&buffer, mDilation);
  serialize_value(&buffer, mDeformableGroup);
  serialize_value(&buffer, mGroup);
}

void ModulatedDeformableConvPluginDynamic::attachToContext(
    cudnnContext *cudnnContext, cublasContext *cublasContext,
    nvinfer1::IGpuAllocator *gpuAllocator) TRT_NOEXCEPT {
  m_cublas_handle = cublasContext;
}

void ModulatedDeformableConvPluginDynamic::detachFromContext() TRT_NOEXCEPT {}

////////////////////// creator /////////////////////////////

ModulatedDeformableConvPluginDynamicCreator::
  ModulatedDeformableConvPluginDynamicCreator() {
  mPluginAttributes.clear();
  mPluginAttributes.emplace_back(nvinfer1::PluginField("stride"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("padding"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("dilation"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("groups"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("deform_groups"));
  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

const char *ModulatedDeformableConvPluginDynamicCreator::getPluginName()
  const TRT_NOEXCEPT {
  return PLUGIN_NAME;
}

const char *ModulatedDeformableConvPluginDynamicCreator::getPluginVersion()
  const TRT_NOEXCEPT {
  return PLUGIN_VERSION;
}

nvinfer1::IPluginV2 *ModulatedDeformableConvPluginDynamicCreator::createPlugin(
    const char *name, const nvinfer1::PluginFieldCollection *fc) TRT_NOEXCEPT {
  nvinfer1::Dims stride{2, {1, 1}};
  nvinfer1::Dims padding{2, {0, 0}};
  nvinfer1::Dims dilation{2, {1, 1}};
  int deformableGroup = 1;
  int group = 1;

  for (int i = 0; i < fc->nbFields; i++) {
    if (fc->fields[i].data == nullptr) {
      continue;
    }
    std::string field_name(fc->fields[i].name);

    if (field_name.compare("deform_groups") == 0) {
      deformableGroup = static_cast<const int *>(fc->fields[i].data)[0];
    }

    if (field_name.compare("groups") == 0) {
      group = static_cast<const int *>(fc->fields[i].data)[0];
    }

    if (field_name.compare("stride") == 0) {
      stride.nbDims = 2;
      stride.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      stride.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }

    if (field_name.compare("padding") == 0) {
      padding.nbDims = 2;
      padding.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      padding.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }

    if (field_name.compare("dilation") == 0) {
      dilation.nbDims = 2;
      dilation.d[0] = static_cast<const int *>(fc->fields[i].data)[0];
      dilation.d[1] = static_cast<const int *>(fc->fields[i].data)[1];
    }
  }

  ModulatedDeformableConvPluginDynamic *plugin =
    new ModulatedDeformableConvPluginDynamic(
      name, stride, padding, dilation, deformableGroup, group);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

nvinfer1::IPluginV2
  *ModulatedDeformableConvPluginDynamicCreator::deserializePlugin(
    const char *name, const void *serialData,
    size_t serialLength) TRT_NOEXCEPT {
  auto plugin =
    new ModulatedDeformableConvPluginDynamic(name, serialData, serialLength);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}
REGISTER_TENSORRT_PLUGIN(ModulatedDeformableConvPluginDynamicCreator);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
