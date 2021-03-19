/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/lib/feature_extractor/tfe/tracking_feat_extractor.h"

#include "cyber/common/file.h"

namespace apollo {
namespace perception {
namespace camera {

bool TrackingFeatureExtractor::Init(
    const FeatureExtractorInitOptions &init_options) {
  //  setup bottom and top
  int feat_height = init_options.feat_blob->shape(2);
  int feat_width = init_options.feat_blob->shape(3);
  input_height_ =
      init_options.input_height == 0 ? feat_height : init_options.input_height;
  input_width_ =
      init_options.input_width == 0 ? feat_width : init_options.input_width;
  tracking_feature::FeatureParam feat_param;
  std::string config_path = cyber::common::GetAbsolutePath(
      init_options.root_dir, init_options.conf_file);
  if (!cyber::common::GetProtoFromFile(config_path, &feat_param)) {
    AERROR << "read proto_config fail";
    return false;
  }
  if (feat_param.extractor_size() != 1) {
    AERROR << "extractor should be 1";
    return false;
  }
  CHECK_EQ(input_height_ / feat_height, input_width_ / feat_width)
      << "Invalid aspect ratio: " << feat_height << "x" << feat_width
      << " from " << input_height_ << "x" << input_width_;

  feat_blob_ = init_options.feat_blob;
  for (int i = 0; i < feat_param.extractor_size(); i++) {
    switch (feat_param.extractor(i).feat_type()) {
      case tracking_feature::ExtractorParam_FeatureType_ROIPooling:
        init_roipooling(init_options,
                        feat_param.extractor(i).roi_pooling_param());
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
    const FeatureExtractorInitOptions &options,
    const tracking_feature::ROIPoolingParam &param) {
  int feat_channel = options.feat_blob->shape(1);
  feat_height_ = options.feat_blob->shape(2);
  feat_width_ = options.feat_blob->shape(3);

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
                                       CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }
  if (frame->detected_objects.empty()) {
    return true;
  }
  if (!options.normalized) {
    encode_bbox(&(frame->detected_objects));
  }
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
    feature_extractor_layer_ptr->pooling_layer->ForwardGPU(
        {feat_blob_, feature_extractor_layer_ptr->rois_blob},
        {frame->track_feature_blob});

    if (!options.normalized) {
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
