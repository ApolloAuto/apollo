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
#include <NvInferVersion.h>

#include "modules/perception/common/inference/tensorrt/rt_common.h"

namespace apollo {
namespace perception {
namespace inference {

// TODO(chenjiahao): complete member functions
#ifdef NV_TENSORRT_MAJOR
#if NV_TENSORRT_MAJOR != 8
class RCNNProposalPlugin : public nvinfer1::IPlugin {
 public:
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

#else
class RCNNProposalPlugin : public nvinfer1::IPluginV2Ext {
 public:
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

  RCNNProposalPlugin() {}
  virtual ~RCNNProposalPlugin() {}

  virtual int initialize() noexcept { return 0; }
  virtual void terminate() noexcept {}
  int32_t getNbOutputs() const noexcept override { return 1; }

  nvinfer1::Dims getOutputDimensions(int32_t index,
      const nvinfer1::Dims *inputs, int32_t nbInputDims)
      noexcept override {
    // TODO(chenjiahao): complete input dims assertion
    // TODO(chenjiahao): batch size is hard coded to 1 here
    return nvinfer1::Dims4(top_n_ * 1, out_channel_, 1, 1);
  }

  void configureWithFormat(const nvinfer1::Dims *inputDims, int32_t nbInputs,
                 const nvinfer1::Dims *outputDims, int32_t nbOutputs,
                 nvinfer1::DataType type, nvinfer1::PluginFormat format,
                 int32_t maxBatchSize) noexcept override {}

  size_t getWorkspaceSize(int32_t maxBatchSize)
      const noexcept override { return 0; }

  int32_t enqueue(int32_t batchSize, const void *const *inputs,
      void *const *outputs, void *workspace, cudaStream_t stream)
      noexcept override;

  size_t getSerializationSize() const noexcept override { return 0; }

  void serialize(void *buffer) const noexcept override {
    char *d = reinterpret_cast<char *>(buffer), *a = d;
    size_t size = getSerializationSize();
    CHECK_EQ(d, a + size);
  }

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
      nvinfer1::PluginFormat format)
      const noexcept override {
    return true;
  }

  void destroy() noexcept override {
    delete this;
  }

  nvinfer1::IPluginV2Ext* clone() const noexcept override {
    RCNNProposalPlugin* p = new RCNNProposalPlugin();
    p->refine_out_of_map_bbox_ = refine_out_of_map_bbox_;
    p->regress_agnostic_ = regress_agnostic_;
    p->rpn_proposal_output_score_ = rpn_proposal_output_score_;

    for (int i = 0; i < 4; i++) {
      p->bbox_mean_[i] = bbox_mean_[i];
    }
    for (int i = 0; i < 4; i++) {
      p->bbox_std_[i] = bbox_std_[i];
    }

    p->min_size_h_ = min_size_h_;
    p->min_size_w_ = min_size_w_;
    p->threshold_objectness_ = threshold_objectness_;
    p->overlap_ratio_ = overlap_ratio_;
    p->num_class_ = num_class_;
    p->num_rois_ = num_rois_;
    p->max_candidate_n_ = max_candidate_n_;
    p->min_size_mode_ = min_size_mode_;
    p->top_n_ = top_n_;
    p->out_channel_ = out_channel_;
    p->acc_box_num_ = acc_box_num_;

    for (size_t i = 0; i < thresholds_.size(); i++) {
      p->thresholds_.push_back(thresholds_[i]);
    }

    p->plugin_namespace = plugin_namespace;
    return p;
  }

  nvinfer1::DataType getOutputDataType(int32_t index,
      nvinfer1::DataType const *inputTypes, int32_t nbInputs)
      const noexcept {
    return nvinfer1::DataType::kFLOAT;
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

  void configurePlugin(
    nvinfer1::Dims const *inputDims, int32_t nbInputs,
    nvinfer1::Dims const *outputDims, int32_t nbOutputs,
    nvinfer1::DataType const *inputTypes,
    nvinfer1::DataType const *outputTypes, bool const *inputIsBroadcast,
    bool const *outputIsBroadcast,
    nvinfer1::PluginFormat floatFormat,
    int32_t maxBatchSize) noexcept {}

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

  nvinfer1::AsciiChar* plugin_namespace;
  const nvinfer1::AsciiChar* plugin_type = "";
  const nvinfer1::AsciiChar* plugin_version = "";
};
#endif
#endif

}  // namespace inference
}  // namespace perception
}  // namespace apollo
