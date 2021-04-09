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

#include "modules/perception/obstacle/camera/detector/common/feature_extractor.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>

#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/perception/obstacle/camera/common/util.h"

namespace apollo {
namespace perception {

using apollo::common::math::L2Norm;

bool ReorgFeatureExtractor::init(
    const ExtractorParam &param,
    const boost::shared_ptr<caffe::Blob<float>> feat_blob, int input_width,
    int input_height) {
  // setup bottom and top
  int feat_height = feat_blob->height();
  int feat_width = feat_blob->width();
  int ref_height = param.reorg_param().ref_height();
  int ref_width = param.reorg_param().ref_width();
  ref_height = ref_height == 0 ? feat_height : ref_height;
  ref_width = ref_width == 0 ? feat_width : ref_width;

  CHECK_EQ(feat_height / ref_height, feat_width / ref_width)
      << "Invalid aspect ratio: " << feat_height << "x" << feat_width
      << ", fb_height=" << feat_blob->height()
      << ", fb_width=" << feat_blob->width() << ", ref_height=" << ref_height
      << ", ref_width=" << ref_width << ", feat_height=" << feat_height
      << ", feat_width=" << feat_width;

  skip_reorg_ = ref_height == feat_height;
  if (skip_reorg_) {
    ADEBUG << "Skip reorg for " << param.feat_blob();
    reorg_feat_blob_ = feat_blob;
    return true;
  }
  reorg_feat_blob_.reset(new caffe::Blob<float>);
  bottom_vec_.push_back(feat_blob.get());
  top_vec_.push_back(reorg_feat_blob_.get());

  // create LayerParameter
  caffe::LayerParameter layer_param;
  layer_param.set_type("Reorg");
  auto *reorg_param = layer_param.mutable_reorg_param();
  int reorg_stride = feat_height / ref_height;
  reorg_param->set_stride(reorg_stride);
  ADEBUG << "Use Reorg: stride=" << reorg_param->stride();
  reorg_layer_ = caffe::LayerRegistry<float>::CreateLayer(layer_param);
  reorg_layer_->SetUp(bottom_vec_, top_vec_);

  ADEBUG << "Shape mismatch: feat_blob=" << feat_blob
         << ", reorg_stride=" << reorg_stride;

  return true;
}

bool ReorgFeatureExtractor::extract(
    std::vector<std::shared_ptr<VisualObject>> *objects) {
  CHECK_NOTNULL(objects);
  if (objects->empty()) {
    return true;
  }
  if (!skip_reorg_) {
    reorg_layer_->Forward(bottom_vec_, top_vec_);
  }

  // get object feature
  for (auto &obj : *objects) {
    int x = (obj->upper_left[0] + obj->lower_right[0]) * 0.5 *
            reorg_feat_blob_->width();
    int y = (obj->upper_left[1] + obj->lower_right[1]) * 0.5 *
            reorg_feat_blob_->height();
    int offset = reorg_feat_blob_->offset(0, 0, y, x);
    int feat_dim = reorg_feat_blob_->channels();
    int spatial_dim = reorg_feat_blob_->count(2);
    ADEBUG << "feat_dim: " << feat_dim << ", spatial_dim: " << spatial_dim;
    const float *feat_data = reorg_feat_blob_->cpu_data() + offset;
    for (int c = 0; c < feat_dim; ++c) {
      obj->object_feature.push_back(feat_data[c * spatial_dim]);
    }
    // feature normalization
    L2Norm(obj->object_feature.size(), obj->object_feature.data());
  }

  return true;
}

bool ROIPoolingFeatureExtractor::init(
    const ExtractorParam &param,
    const boost::shared_ptr<caffe::Blob<float>> feat_blob, int input_width,
    int input_height) {
  // setup bottom and top
  int feat_height = feat_blob->height();
  int feat_width = feat_blob->width();
  input_height_ = input_height == 0 ? feat_height : input_height;
  input_width_ = input_width == 0 ? feat_width : input_width;

  CHECK_EQ(input_height_ / feat_height, input_width_ / feat_width)
      << "Invalid aspect ratio: " << feat_height << "x" << feat_width;

  bottom_vec_.push_back(feat_blob.get());
  bottom_vec_.push_back(&rois_blob_);
  top_vec_.push_back(&roi_feat_blob_);

  // create LayerParameter
  caffe::LayerParameter layer_param;
  layer_param.set_type("ROIPooling");
  auto *rp_param = layer_param.mutable_roi_pooling_param();
  rp_param->set_pooled_h(param.roi_pooling_param().pooled_h());
  rp_param->set_pooled_w(param.roi_pooling_param().pooled_w());
  rp_param->set_use_floor(param.roi_pooling_param().use_floor());
  rp_param->set_spatial_scale(static_cast<float>(feat_height) / input_height_);

  ADEBUG << "Use ROIPooling: pooled_h=" << rp_param->pooled_h()
         << ", pooled_w=" << rp_param->pooled_w()
         << ", spatial_scale=" << rp_param->spatial_scale();
  roi_pooling_layer_ = caffe::LayerRegistry<float>::CreateLayer(layer_param);
  rois_blob_.Reshape({1, 5});
  roi_pooling_layer_->SetUp(bottom_vec_, top_vec_);
  return true;
}

bool ROIPoolingFeatureExtractor::extract(
    std::vector<std::shared_ptr<VisualObject>> *objects) {
  CHECK_NOTNULL(objects);
  if (objects->empty()) {
    return true;
  }
  rois_blob_.Reshape({static_cast<int>(objects->size()), 5});
  float *rois_data = rois_blob_.mutable_cpu_data();
  for (const auto &obj : *objects) {
    rois_data[0] = 0;
    rois_data[1] = obj->upper_left[0] * input_width_;
    rois_data[2] = obj->upper_left[1] * input_height_;
    rois_data[3] = obj->lower_right[0] * input_width_;
    rois_data[4] = obj->lower_right[1] * input_height_;
    ADEBUG << rois_data[0] << " " << rois_data[1] << " " << rois_data[2] << " "
           << rois_data[3] << " " << rois_data[4];
    rois_data += rois_blob_.offset(1);
  }
  roi_pooling_layer_->Forward(bottom_vec_, top_vec_);
  int feat_dim = roi_feat_blob_.count() / objects->size();
  const float *feat_data = roi_feat_blob_.cpu_data();
  for (const auto &obj : *objects) {
    obj->object_feature.resize(feat_dim);
    memcpy(obj->object_feature.data(), feat_data,
           feat_dim * sizeof(feat_data[0]));
    L2Norm(feat_dim, obj->object_feature.data());
    feat_data += feat_dim;
  }
  return true;
}

}  // namespace perception
}  // namespace apollo
