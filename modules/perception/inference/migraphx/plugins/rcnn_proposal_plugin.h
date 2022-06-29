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

#include <migraphx/argument.hpp>

#include "modules/perception/inference/migraphx/mi_common.h"
#include "modules/perception/inference/migraphx/plugins/plugin.h"

namespace apollo {
namespace perception {
namespace inference {

// TODO(chenjiahao): complete member functions
class RCNNProposalPlugin : public nvinfer1::IPlugin {
 public:
  RCNNProposalPlugin() {}
  template <class T, class F>
  static auto reflect(T &self, F f) {
    return migraphx::pack(
        // f(self.thread_size_, "thread_size_"),
        f(self.refine_out_of_map_bbox_, "refine_out_of_map_bbox_"),
        f(self.regress_agnostic_, "regress_agnostic_"),
        f(self.rpn_proposal_output_score_, "rpn_proposal_output_score_"),
        f(self.bbox_mean_, "bbox_mean_"), f(self.bbox_std_, "bbox_std_"),
        f(self.min_size_h_, "min_size_h_"), f(self.min_size_w_, "min_size_w_"),
        f(self.threshold_objectness_, "threshold_objectness_"),
        f(self.overlap_ratio_, "overlap_ratio_"),
        f(self.num_class_, "num_class_"), f(self.num_rois_, "num_rois_"),
        f(self.max_candidate_n_, "max_candidate_n_"),
        f(self.min_size_mode_, "min_size_mode_"), f(self.top_n_, "top_n_"),
        f(self.out_channel_, "out_channel_"),
        f(self.acc_box_num_, "acc_box_num_"),
        f(self.thresholds_, "thresholds_"));
  }

  friend std::ostream &operator<<(std::ostream &ss,
                                  const RCNNProposalPlugin &d) {
    ss << "{ refine_out_of_map_bbox_: " << d.refine_out_of_map_bbox_
       << ", regress_agnostic_: " << d.regress_agnostic_
       << ", rpn_proposal_output_score_: "
       << d.rpn_proposal_output_score_
       // << ", bbox_mean_: " d.<< bbox_mean_
       // << ", bbox_std_: " d.<< bbox_std_
       << ", min_size_h_: " << d.min_size_h_
       << ", min_size_w_: " << d.min_size_w_
       << ", threshold_objectness_: " << d.threshold_objectness_
       << ", overlap_ratio_: " << d.overlap_ratio_
       << ", num_class_: " << d.num_class_ << ", num_rois_: " << d.num_rois_
       << ", max_candidate_n_: " << d.max_candidate_n_
       << ", min_size_mode_: " << d.min_size_mode_ << ", top_n_: " << d.top_n_
       << ", out_channel_: " << d.out_channel_ << ", acc_box_num_: "
       << d.acc_box_num_
       // << ", thresholds_: " << d.thresholds_
       << " }";
    return ss;
  }

  friend bool operator==(const RCNNProposalPlugin &lhs,
                         const RCNNProposalPlugin &rhs) {
    return lhs.refine_out_of_map_bbox_ == rhs.refine_out_of_map_bbox_ &&
           lhs.regress_agnostic_ == rhs.regress_agnostic_ &&
           lhs.rpn_proposal_output_score_ == rhs.rpn_proposal_output_score_ &&
           lhs.bbox_mean_ == rhs.bbox_mean_ && lhs.bbox_std_ == rhs.bbox_std_ &&
           lhs.min_size_h_ == rhs.min_size_h_ &&
           lhs.min_size_w_ == rhs.min_size_w_ &&
           lhs.threshold_objectness_ == rhs.threshold_objectness_ &&
           lhs.overlap_ratio_ == rhs.overlap_ratio_ &&
           lhs.num_class_ == rhs.num_class_ && lhs.num_rois_ == rhs.num_rois_ &&
           lhs.max_candidate_n_ == rhs.max_candidate_n_ &&
           lhs.min_size_mode_ == rhs.min_size_mode_ &&
           lhs.top_n_ == rhs.top_n_ && lhs.out_channel_ == rhs.out_channel_ &&
           lhs.acc_box_num_ == rhs.acc_box_num_ &&
           lhs.thresholds_ == rhs.thresholds_;
  }

  RCNNProposalPlugin &operator=(const RCNNProposalPlugin &other) {
    if (this == &other) {
      return *this;
    }

    this->refine_out_of_map_bbox_ = other.refine_out_of_map_bbox_;
    this->regress_agnostic_ = other.regress_agnostic_;
    this->rpn_proposal_output_score_ = other.rpn_proposal_output_score_;
    this->bbox_mean_ = other.bbox_mean_;
    this->bbox_std_ = other.bbox_std_;
    this->min_size_h_ = other.min_size_h_;
    this->min_size_w_ = other.min_size_w_;
    this->threshold_objectness_ = other.threshold_objectness_;
    this->overlap_ratio_ = other.overlap_ratio_;
    this->num_class_ = other.num_class_;
    this->num_rois_ = other.num_rois_;
    this->max_candidate_n_ = other.max_candidate_n_;
    this->min_size_mode_ = other.min_size_mode_;
    this->top_n_ = other.top_n_;
    this->out_channel_ = other.out_channel_;
    this->acc_box_num_ = other.acc_box_num_;
    this->thresholds_ = other.thresholds_;

    return *this;
  }

  RCNNProposalPlugin(
      const BBoxRegParameter &bbox_reg_param,
      const DetectionOutputSSDParameter &detection_output_ssd_param,
      nvinfer1::Dims *in_dims) {
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

  virtual int initialize() { return 0; }
  virtual void terminate() {}
  int getNbOutputs() const override { return 1; }

  nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims *inputs,
                                     int nbInputDims) override {
    // TODO(chenjiahao): complete input dims assertion
    // TODO(chenjiahao): batch size is hard coded to 1 here
    return nvinfer1::Dims4(top_n_ * 1, out_channel_, 1, 1);
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
  bool refine_out_of_map_bbox_ = true;
  // TODO(chenjiahao): implement class-agnostic regression option
  bool regress_agnostic_ = false;
  bool rpn_proposal_output_score_ = true;

  // float bbox_mean_[4];
  // float bbox_std_[4];
  std::vector<float> bbox_mean_ = {
      0.0,
      0.0,
      0.0,
      0.0,
  };
  std::vector<float> bbox_std_ = {
      0.0,
      0.0,
      0.0,
      0.0,
  };
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
