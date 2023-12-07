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

#include "cyber/common/log.h"
#include "modules/perception/common/inference/migraphx/mi_common.h"
#include "modules/perception/common/inference/migraphx/plugins/plugin.h"

namespace apollo {
namespace perception {
namespace inference {

// TODO(chenjiahao): complete member functions
// Custom layer for DFMBPSROIAlign operation,
// i.e. DeForMaBle Position Sensitive ROI Align.
// input0 dims: [C, H, W], input1 dims: [num_rois, 5, 1, 1]
// input2 dims: [N, C2, H2, W2]
class DFMBPSROIAlignPlugin : public IPlugin {
 public:
  DFMBPSROIAlignPlugin(
      const DFMBPSROIAlignParameter &dfmb_psroi_align_parameter, Dims *in_dims,
      int nbInputs) {
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
    output_dims_ =
        Dims4(in_dims[1].d[0], output_channel_, pooled_height_, pooled_width_);
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

  int initialize() override { return 0; }
  void terminate() override {}
  int getNbOutputs() const override { return 1; }

  Dims getOutputDimensions(int index, const Dims *inputs,
                           int nbInputDims) override {
    // TODO(chenjiahao): complete input dims assertion
    return output_dims_;
  }

  void configure(const Dims *inputDims, int nbInputs, const Dims *outputDims,
                 int nbOutputs, int maxBatchSize) override {}

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

  Dims output_dims_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
