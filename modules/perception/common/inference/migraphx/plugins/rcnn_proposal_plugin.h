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

#include <vector>

#include "modules/perception/common/inference/migraphx/mi_common.h"
#include "modules/perception/common/inference/migraphx/plugins/plugin.h"

namespace apollo {
namespace perception {
namespace inference {

// TODO(chenjiahao): complete member functions
class RCNNProposalPlugin : public IPlugin {
 public:
  RCNNProposalPlugin(
      const BBoxRegParameter &bbox_reg_param,
      const DetectionOutputSSDParameter &detection_output_ssd_param,
      Dims *in_dims) {
    num_rois_ = in_dims[2].d[0];

    for (int i = 0; i < 4; ++i) {
      bbox_mean_[i] = bbox_reg_param.bbox_mean(i);
      bbox_std_[i] = bbox_reg_param.bbox_std(i);
    }

    min_size_mode_ =
        static_cast<int>(detection_output_ssd_param.min_size_mode());
    min_size_h_ = detection_output_ssd_param.min_size_h();
    min_size_w_ = detection_output_ssd_param.min_size_w();

    num_class_ = detection_output_ssd_param.num_class();
    refine_out_of_map_bbox_ =
        detection_output_ssd_param.refine_out_of_map_bbox();
    regress_agnostic_ = detection_output_ssd_param.regress_agnostic();
    rpn_proposal_output_score_ =
        detection_output_ssd_param.rpn_proposal_output_score();

    threshold_objectness_ = detection_output_ssd_param.threshold_objectness();
    for (int i = 0; i < num_class_; ++i) {
      thresholds_.push_back(detection_output_ssd_param.threshold(i));
    }

    NMSSSDParameter nms_param = detection_output_ssd_param.nms_param();
    max_candidate_n_ = nms_param.max_candidate_n(0);
    overlap_ratio_ = nms_param.overlap_ratio(0);
    top_n_ = nms_param.top_n(0);

    out_channel_ = rpn_proposal_output_score_ ? 9 : 5;
  }

  virtual ~RCNNProposalPlugin() {}

  int initialize() override { return 0; }
  void terminate() override {}
  int getNbOutputs() const override { return 1; }

  Dims getOutputDimensions(int index, const Dims *inputs,
                           int nbInputDims) override {
    // TODO(chenjiahao): complete input dims assertion
    // TODO(chenjiahao): batch size is hard coded to 1 here
    return Dims4(top_n_ * 1, out_channel_, 1, 1);
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
  bool refine_out_of_map_bbox_ = true;
  // TODO(chenjiahao): implement class-agnostic regression option
  bool regress_agnostic_ = false;
  bool rpn_proposal_output_score_ = true;

  float bbox_mean_[4];
  float bbox_std_[4];
  float min_size_h_;
  float min_size_w_;
  float threshold_objectness_;
  float overlap_ratio_;
  int num_class_;
  int num_rois_;
  int max_candidate_n_;
  int min_size_mode_;
  int top_n_;
  int out_channel_;
  int acc_box_num_;

  std::vector<float> thresholds_{};
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo
