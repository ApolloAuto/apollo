/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera_tracking/feature_extract/tracking_feat_extractor.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

bool TrackingFeatureExtractor::Init(
    const FeatureExtractorInitOptions &options) {
  tracking_feature::FeatureParam feat_param;
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  if (!cyber::common::GetProtoFromFile(config_file, &feat_param)) {
    AERROR << "Read feature extractor config file failed!";
    return false;
  }
  if (feat_param.extractor_size() != 1) {
    AERROR << "extractor should be 1";
    return false;
  }

  const std::string feat_blob_name = feat_param.feat_blob_name();
  const auto &feat_blob_shape_pb = feat_param.feat_blob_shape();

  std::vector<int> feat_blob_shape(feat_blob_shape_pb.begin(),
                                   feat_blob_shape_pb.end());
  feat_blob_ =
      std::make_shared<apollo::perception::base::Blob<float>>(feat_blob_shape);

  //  setup bottom and top
  int feat_height = feat_blob_->shape(2);
  int feat_width = feat_blob_->shape(3);

  input_height_ =
      options.input_height == 0 ? feat_height : options.input_height;
  input_width_ = options.input_width == 0 ? feat_width : options.input_width;

  CHECK_EQ(input_height_ / feat_height, input_width_ / feat_width)
      << "Invalid aspect ratio: " << feat_height << "x" << feat_width
      << " from " << input_height_ << "x" << input_width_;
  for (int i = 0; i < feat_param.extractor_size(); i++) {
    switch (feat_param.extractor(i).feat_type()) {
      case tracking_feature::ExtractorParam_FeatureType_ROIPooling:
        init_roipooling(feat_param.extractor(i).roi_pooling_param());
        break;
    }
  }
  if (roi_poolings_.empty()) {
    AERROR << "no proper extractor";
    return false;
  }
  return true;
}

void TrackingFeatureExtractor::init_roipooling(
    const tracking_feature::ROIPoolingParam &param) {
  int feat_channel = feat_blob_->shape(1);
  feat_height_ = feat_blob_->shape(2);
  feat_width_ = feat_blob_->shape(3);

  std::shared_ptr<FeatureExtractorLayer> feature_extractor_layer_ptr;
  feature_extractor_layer_ptr.reset(new FeatureExtractorLayer());
  std::vector<int> shape{1, 5};
  feature_extractor_layer_ptr->rois_blob.reset(new base::Blob<float>(shape));
  int pooled_w = param.pooled_w();
  int pooled_h = param.pooled_h();
  bool use_floor = param.use_floor();
  feature_extractor_layer_ptr->pooling_layer.reset(
      new inference::ROIPoolingLayer<float>(pooled_h, pooled_w, use_floor, 1,
                                            feat_channel));
  feature_extractor_layer_ptr->top_blob.reset(
      new base::Blob<float>(1, feat_blob_->channels(), pooled_h, pooled_w));
  roi_poolings_.push_back(feature_extractor_layer_ptr);
}

bool TrackingFeatureExtractor::Extract(const FeatureExtractorOptions &options,
                                       CameraTrackingFrame *frame) {
  if (frame == nullptr) {
    return false;
  }
  if (frame->detected_objects.empty()) {
    return true;
  }
  if (!options.normalized) {
    // normalize bbox
    encode_bbox(&(frame->detected_objects));
  }
  auto feat_blob = frame->feature_blob;
  for (auto feature_extractor_layer_ptr : roi_poolings_) {
    feature_extractor_layer_ptr->rois_blob->Reshape(
        {static_cast<int>(frame->detected_objects.size()), 5});
    float *rois_data =
        feature_extractor_layer_ptr->rois_blob->mutable_cpu_data();
    for (const auto &obj : frame->detected_objects) {
      rois_data[0] = 0;
      rois_data[1] =
          obj->camera_supplement.box.xmin * static_cast<float>(feat_width_);
      rois_data[2] =
          obj->camera_supplement.box.ymin * static_cast<float>(feat_height_);
      rois_data[3] =
          obj->camera_supplement.box.xmax * static_cast<float>(feat_width_);
      rois_data[4] =
          obj->camera_supplement.box.ymax * static_cast<float>(feat_height_);
      ADEBUG << rois_data[0] << " " << rois_data[1] << " " << rois_data[2]
             << " " << rois_data[3] << " " << rois_data[4];
      rois_data += feature_extractor_layer_ptr->rois_blob->offset(1);
    }

    // obtain the track feature blob
    feature_extractor_layer_ptr->pooling_layer->ForwardGPU(
        {feat_blob, feature_extractor_layer_ptr->rois_blob},
        {frame->track_feature_blob});

    if (!options.normalized) {
      // denormalize bbox
      decode_bbox(&(frame->detected_objects));
    }
  }
  norm_.L2Norm(frame->track_feature_blob.get());
  return true;
}

REGISTER_FEATURE_EXTRACTOR(TrackingFeatureExtractor);
}  // namespace camera
}  // namespace perception
}  // namespace apollo
