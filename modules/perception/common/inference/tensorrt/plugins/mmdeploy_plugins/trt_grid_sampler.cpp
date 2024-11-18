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

#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_grid_sampler.hpp"

#include <assert.h>

#include <chrono>

#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_grid_sampler_kernel.hpp"
#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_plugin_helper.hpp"
#include "modules/perception/common/inference/tensorrt/plugins/mmdeploy_plugins/trt_serialize.hpp"

namespace apollo {
namespace perception {
namespace inference {

namespace {
static const char *PLUGIN_VERSION{"1"};
static const char *PLUGIN_NAME{"grid_sampler"};
}  // namespace

TRTGridSampler::TRTGridSampler(const std::string &name, int mode,
                               int paddingMode, bool alignCorners)
    : TRTPluginBase(name), mMode(mode), mPaddingMode(paddingMode),
      mAlignCorners(alignCorners) {}

TRTGridSampler::TRTGridSampler(
  const std::string name, const void *data, size_t length)
    : TRTPluginBase(name) {
  deserialize_value(&data, &length, &mMode);
  deserialize_value(&data, &length, &mPaddingMode);
  deserialize_value(&data, &length, &mAlignCorners);
}

nvinfer1::IPluginV2DynamicExt *TRTGridSampler::clone() const TRT_NOEXCEPT {
  TRTGridSampler *plugin = new TRTGridSampler(mLayerName, mMode,
    mPaddingMode, mAlignCorners);
  plugin->setPluginNamespace(getPluginNamespace());

  return plugin;
}

nvinfer1::DimsExprs TRTGridSampler::getOutputDimensions(
    int outputIndex, const nvinfer1::DimsExprs *inputs, int nbInputs,
    nvinfer1::IExprBuilder &exprBuilder) TRT_NOEXCEPT {
  nvinfer1::DimsExprs ret;
  ret.nbDims = inputs[0].nbDims;
  ret.d[0] = inputs[0].d[0];
  ret.d[1] = inputs[0].d[1];
  for (int i = 2; i < ret.nbDims; ++i) {
    ret.d[i] = inputs[1].d[i - 1];
  }
  return ret;
}

bool TRTGridSampler::supportsFormatCombination(
  int pos, const nvinfer1::PluginTensorDesc *ioDesc,
  int nbInputs, int nbOutputs) TRT_NOEXCEPT {
  if (pos == 0) {
    return (ioDesc[pos].type == nvinfer1::DataType::kFLOAT &&
            ioDesc[pos].format == nvinfer1::TensorFormat::kLINEAR);
  } else {
    return ioDesc[pos].type == ioDesc[0].type &&
           ioDesc[pos].format == ioDesc[0].format;
  }
}

void TRTGridSampler::configurePlugin(
  const nvinfer1::DynamicPluginTensorDesc *inputs, int nbInputs,
  const nvinfer1::DynamicPluginTensorDesc *outputs,
  int nbOutputs) TRT_NOEXCEPT {
  // Validate input arguments
}

size_t TRTGridSampler::getWorkspaceSize(
  const nvinfer1::PluginTensorDesc *inputs, int nbInputs,
  const nvinfer1::PluginTensorDesc *outputs,
  int nbOutputs) const TRT_NOEXCEPT {
  return 0;
}

int TRTGridSampler::enqueue(const nvinfer1::PluginTensorDesc *inputDesc,
                            const nvinfer1::PluginTensorDesc *outputDesc,
                            const void *const *inputs, void *const *outputs,
                            void *workSpace, cudaStream_t stream) TRT_NOEXCEPT {
  nvinfer1::Dims input_dims = inputDesc[0].dims;
  nvinfer1::Dims grid_dims = inputDesc[1].dims;
  nvinfer1::Dims output_dims = outputDesc[0].dims;

  GridSamplerInterpolation interp_mode = GridSamplerInterpolation::Bilinear;
  switch (mMode) {
    case 0:
      interp_mode = GridSamplerInterpolation::Bilinear;
      break;
    case 1:
      interp_mode = GridSamplerInterpolation::Nearest;
      break;
    default:
      break;
  }

  GridSamplerPadding padding_mode = GridSamplerPadding::Zeros;
  switch (mPaddingMode) {
    case 0:
      padding_mode = GridSamplerPadding::Zeros;
      break;

    case 1:
      padding_mode = GridSamplerPadding::Border;
      break;

    case 2:
      padding_mode = GridSamplerPadding::Reflection;
      break;
    default:
      break;
  }

  auto data_type = inputDesc[0].type;

  switch (data_type) {
    case nvinfer1::DataType::kFLOAT:
      grid_sample<float>(reinterpret_cast<float*>(outputs[0]),
                         reinterpret_cast<const float*>(inputs[0]),
                         reinterpret_cast<const float*>(inputs[1]),
                         &(output_dims.d[0]),
                         &(input_dims.d[0]), &(grid_dims.d[0]),
                         input_dims.nbDims, interp_mode,
                         padding_mode, mAlignCorners, stream);
      break;
    default:
      return 1;
      break;
  }

  return 0;
}

nvinfer1::DataType TRTGridSampler::getOutputDataType(
  int index, const nvinfer1::DataType *inputTypes,
  int nbInputs) const TRT_NOEXCEPT {
  return inputTypes[0];
}

// IPluginV2 Methods
const char *TRTGridSampler::getPluginType() const TRT_NOEXCEPT
  { return PLUGIN_NAME; }

const char *TRTGridSampler::getPluginVersion() const TRT_NOEXCEPT
  { return PLUGIN_VERSION; }

int TRTGridSampler::getNbOutputs() const TRT_NOEXCEPT
  { return 1; }

size_t TRTGridSampler::getSerializationSize() const TRT_NOEXCEPT {
  return serialized_size(mMode) + serialized_size(mPaddingMode) +
         serialized_size(mAlignCorners);
}

void TRTGridSampler::serialize(void *buffer) const TRT_NOEXCEPT {
  serialize_value(&buffer, mMode);
  serialize_value(&buffer, mPaddingMode);
  serialize_value(&buffer, mAlignCorners);
}

////////////////////// creator /////////////////////////////

TRTGridSamplerCreator::TRTGridSamplerCreator() {
  mPluginAttributes = std::vector<nvinfer1::PluginField>(
      {nvinfer1::PluginField("interpolation_mode"),
       nvinfer1::PluginField("padding_mode"),
       nvinfer1::PluginField("align_corners")});
  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

const char *TRTGridSamplerCreator::getPluginName() const TRT_NOEXCEPT
  { return PLUGIN_NAME; }

const char *TRTGridSamplerCreator::getPluginVersion() const TRT_NOEXCEPT
  { return PLUGIN_VERSION; }

nvinfer1::IPluginV2 *TRTGridSamplerCreator::createPlugin(
    const char *name, const nvinfer1::PluginFieldCollection *fc) TRT_NOEXCEPT {
  int mode = 0;
  int paddingMode = 0;
  bool alignCorners = false;

  for (int i = 0; i < fc->nbFields; i++) {
    if (fc->fields[i].data == nullptr) {
      continue;
    }
    std::string field_name(fc->fields[i].name);

    if (field_name.compare("interpolation_mode") == 0) {
      mode = static_cast<const int *>(fc->fields[i].data)[0];
    }

    if (field_name.compare("padding_mode") == 0) {
      paddingMode = static_cast<const int *>(fc->fields[i].data)[0];
    }

    if (field_name.compare("align_corners") == 0) {
      alignCorners =
        static_cast<bool>((static_cast<const int *>(fc->fields[i].data)[0]));
    }
  }

  TRTGridSampler *plugin =
    new TRTGridSampler(name, mode, paddingMode, alignCorners);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

nvinfer1::IPluginV2 *TRTGridSamplerCreator::deserializePlugin(
  const char *name, const void *serialData, size_t serialLength) TRT_NOEXCEPT {
  // This object will be deleted when the network is destroyed, which will
  // call FCPluginDynamic::destroy()
  auto plugin = new TRTGridSampler(name, serialData, serialLength);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

REGISTER_TENSORRT_PLUGIN(TRTGridSamplerCreator);

}  // namespace inference
}  // namespace perception
}  // namespace apollo
