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

#pragma once

#include <NvInferVersion.h>

#include "cyber/common/log.h"
#include "modules/perception/common/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

// TODO(chenjiahao): complete member functions
// Custom layer for DFMBPSROIAlign operation,
// i.e. DeForMaBle Position Sensitive ROI Align.
// input0 dims: [C, H, W], input1 dims: [num_rois, 5, 1, 1]
// input2 dims: [N, C2, H2, W2]
#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR != 8
class DFMBPSROIAlignPlugin : public nvinfer1::IPlugin {
 public:
  DFMBPSROIAlignPlugin(
      const DFMBPSROIAlignParameter &dfmb_psroi_align_parameter,
      nvinfer1::Dims *in_dims, int nbInputs) {
    heat_map_a_ = dfmb_psroi_align_parameter.heat_map_a();
    output_channel_ = dfmb_psroi_align_parameter.output_dim();
    group_height_ = dfmb_psroi_align_parameter.group_height();
    group_width_ = dfmb_psroi_align_parameter.group_width();
    pooled_height_ = dfmb_psroi_align_parameter.pooled_height();
    pooled_width_ = dfmb_psroi_align_parameter.pooled_width();
    pad_ratio_ = dfmb_psroi_align_parameter.pad_ratio();
    sample_per_part_ = dfmb_psroi_align_parameter.sample_per_part();

    trans_std_ = dfmb_psroi_align_parameter.trans_std();
    part_height_ = dfmb_psroi_align_parameter.part_height();
    part_width_ = dfmb_psroi_align_parameter.part_width();
    heat_map_b_ = dfmb_psroi_align_parameter.heat_map_b();
    no_trans_ = (nbInputs < 3);
    num_classes_ = no_trans_ ? 1 : in_dims[2].d[1];

    CHECK_GT(heat_map_a_, 0);
    CHECK_GE(heat_map_b_, 0);
    CHECK_GE(pad_ratio_, 0);
    CHECK_GT(output_channel_, 0);
    CHECK_GT(sample_per_part_, 0);
    CHECK_GT(group_height_, 0);
    CHECK_GT(group_width_, 0);
    CHECK_GT(pooled_height_, 0);
    CHECK_GT(pooled_width_, 0);
    CHECK_GE(part_height_, 0);
    CHECK_GE(part_width_, 0);

    channels_ = in_dims[0].d[0];
    height_ = in_dims[0].d[1];
    width_ = in_dims[0].d[2];
    output_dims_ = nvinfer1::Dims4(in_dims[1].d[0], output_channel_,
                                   pooled_height_, pooled_width_);
    output_size_ =
        in_dims[1].d[0] * output_channel_ * pooled_height_ * pooled_width_;

    CHECK_EQ(channels_, output_channel_ * group_height_ * group_width_);
    CHECK_EQ(in_dims[1].d[1], 5);
    if (!no_trans_) {
      CHECK_EQ(in_dims[2].d[1] % 2, 0);
      int num_classes = in_dims[2].d[1] / 2;
      CHECK_EQ(output_channel_ % num_classes, 0);
      CHECK_EQ(part_height_, in_dims[2].d[2]);
      CHECK_EQ(part_width_, in_dims[2].d[3]);
    }
  }

  virtual ~DFMBPSROIAlignPlugin() {}

  virtual int initialize() { return 0; }
  virtual void terminate() {}
  int getNbOutputs() const override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) override {
    // TODO(chenjiahao): complete input dims assertion
    return output_dims_;
  }

  void configure(const nvinfer1::Dims *inputDims, int nbInputs,
                 const nvinfer1::Dims *outputDims, int nbOutputs,
                 int maxBatchSize) override {}

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
  const int thread_size_ = 512;
  float heat_map_a_;
  float heat_map_b_;
  float pad_ratio_;

  int output_channel_;
  bool no_trans_;
  float trans_std_;
  int sample_per_part_;
  int group_height_;
  int group_width_;
  int pooled_height_;
  int pooled_width_;
  int part_height_;
  int part_width_;
  int num_classes_;

  int channels_;
  int height_;
  int width_;
  int output_size_;

  nvinfer1::Dims output_dims_;
};

#else
class DFMBPSROIAlignPlugin : public nvinfer1::IPluginV2Ext {
 public:
  DFMBPSROIAlignPlugin(
      const DFMBPSROIAlignParameter &dfmb_psroi_align_parameter,
      nvinfer1::Dims *in_dims, int nbInputs) {
    heat_map_a_ = dfmb_psroi_align_parameter.heat_map_a();
    output_channel_ = dfmb_psroi_align_parameter.output_dim();
    group_height_ = dfmb_psroi_align_parameter.group_height();
    group_width_ = dfmb_psroi_align_parameter.group_width();
    pooled_height_ = dfmb_psroi_align_parameter.pooled_height();
    pooled_width_ = dfmb_psroi_align_parameter.pooled_width();
    pad_ratio_ = dfmb_psroi_align_parameter.pad_ratio();
    sample_per_part_ = dfmb_psroi_align_parameter.sample_per_part();

    trans_std_ = dfmb_psroi_align_parameter.trans_std();
    part_height_ = dfmb_psroi_align_parameter.part_height();
    part_width_ = dfmb_psroi_align_parameter.part_width();
    heat_map_b_ = dfmb_psroi_align_parameter.heat_map_b();
    no_trans_ = (nbInputs < 3);
    num_classes_ = no_trans_ ? 1 : in_dims[2].d[1];

    CHECK_GT(heat_map_a_, 0);
    CHECK_GE(heat_map_b_, 0);
    CHECK_GE(pad_ratio_, 0);
    CHECK_GT(output_channel_, 0);
    CHECK_GT(sample_per_part_, 0);
    CHECK_GT(group_height_, 0);
    CHECK_GT(group_width_, 0);
    CHECK_GT(pooled_height_, 0);
    CHECK_GT(pooled_width_, 0);
    CHECK_GE(part_height_, 0);
    CHECK_GE(part_width_, 0);

    channels_ = in_dims[0].d[0];
    height_ = in_dims[0].d[1];
    width_ = in_dims[0].d[2];
    output_dims_ = nvinfer1::Dims4(in_dims[1].d[0], output_channel_,
                                   pooled_height_, pooled_width_);
    output_size_ =
        in_dims[1].d[0] * output_channel_ * pooled_height_ * pooled_width_;

    CHECK_EQ(channels_, output_channel_ * group_height_ * group_width_);
    CHECK_EQ(in_dims[1].d[1], 5);
    if (!no_trans_) {
      CHECK_EQ(in_dims[2].d[1] % 2, 0);
      int num_classes = in_dims[2].d[1] / 2;
      CHECK_EQ(output_channel_ % num_classes, 0);
      CHECK_EQ(part_height_, in_dims[2].d[2]);
      CHECK_EQ(part_width_, in_dims[2].d[3]);
    }
  }

  DFMBPSROIAlignPlugin() {}
  virtual ~DFMBPSROIAlignPlugin() {}

  virtual int initialize() noexcept { return 0; }
  virtual void terminate() noexcept {}
  int32_t getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int32_t index,
      const nvinfer1::Dims *inputs,
      int32_t nbInputDims) noexcept override {
    // TODO(chenjiahao): complete input dims assertion
    return output_dims_;
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims, int32_t nbInputs,
                 const nvinfer1::Dims *outputDims, int32_t nbOutputs,
                 nvinfer1::DataType type, nvinfer1::PluginFormat format,
                 int32_t maxBatchSize) noexcept override {}

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

  nvinfer1::AsciiChar const* getPluginType() const noexcept override {
    return plugin_type;
  }

  nvinfer1::AsciiChar const* getPluginVersion() const noexcept override {
    return plugin_version;
  }

  void setPluginNamespace(const nvinfer1::AsciiChar* libNamespace)
      noexcept override {
    plugin_namespace = const_cast<nvinfer1::AsciiChar*>(libNamespace);
  }

  nvinfer1::AsciiChar const* getPluginNamespace() const noexcept override {
    return const_cast<nvinfer1::AsciiChar*>(plugin_namespace);
  }

  bool supportsFormat(nvinfer1::DataType type,
      nvinfer1::PluginFormat format) const noexcept override {
    return true;
  }

  void destroy() noexcept override {
    delete this;
  }

  bool isOutputBroadcastAcrossBatch(int32_t outputIndex,
      bool const *inputIsBroadcasted,
      int32_t nbInputs) const noexcept override {
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

  nvinfer1::IPluginV2Ext* clone() const noexcept override {
    DFMBPSROIAlignPlugin* p = new DFMBPSROIAlignPlugin();
    p->heat_map_a_ = heat_map_a_;
    p->heat_map_b_ = heat_map_b_;
    p->pad_ratio_ = pad_ratio_;

    p->output_channel_ = output_channel_;
    p->no_trans_ = no_trans_;
    p->trans_std_ = trans_std_;
    p->sample_per_part_ = sample_per_part_;
    p->group_height_ = group_height_;
    p->group_width_ = group_width_;
    p->pooled_height_ = pooled_height_;
    p->pooled_width_ = pooled_width_;
    p->part_height_ = part_height_;
    p->part_width_ = part_width_;
    p->num_classes_ = num_classes_;

    p->channels_ = channels_;
    p->height_ = height_;
    p->width_ = width_;
    p->output_size_ = output_size_;

    p->output_dims_ = nvinfer1::Dims4(
      output_size_ / (output_channel_ * pooled_height_ * pooled_width_),
      output_channel_, pooled_height_, pooled_width_);

    p->plugin_namespace = plugin_namespace;
    return p;
  }

 private:
  const int thread_size_ = 512;
  float heat_map_a_;
  float heat_map_b_;
  float pad_ratio_;

  int output_channel_;
  bool no_trans_;
  float trans_std_;
  int sample_per_part_;
  int group_height_;
  int group_width_;
  int pooled_height_;
  int pooled_width_;
  int part_height_;
  int part_width_;
  int num_classes_;

  int channels_;
  int height_;
  int width_;
  int output_size_;

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
