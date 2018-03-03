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

#include "modules/obstacle/camera/detector/common/tracking_feature.pb.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/util.h"
#include "modules/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"

#include "modules/lib/base/file_util.h"
#include "modules/obstacle/camera/common/flags.h"
#include "modules/obstacle/camera/common/util.h"

namespace apollo {
namespace perception {
namespace obstacle {

DECLARE_int32(obs_camera_detector_gpu);

bool YoloCameraDetector::init(const CameraDetectorInitOptions &options) {
  config_manager::ConfigManager *config_manager =
      base::Singleton<config_manager::ConfigManager>::get();
  std::string model_name = this->name();
  const config_manager::ModelConfig *model_config = NULL;
  if (!config_manager->get_model_config(model_name, &model_config)) {
    AERROR << "not found model: " << model_name;
    return false;
  }
  std::string work_root = config_manager->work_root();

  std::string yolo_root;
  if (!model_config->get_value("yolo_root", &yolo_root)) {
    AERROR << "yolo_root not found.";
    return false;
  }
  yolo_root = base::FileUtil::get_absolute_path(work_root, yolo_root);

  std::string yolo_config =
      base::FileUtil::get_absolute_path(yolo_root, "config.pt");
  load_text_proto_message_file(yolo_config, yolo_param_);
  load_intrinsic(options);
  if (!init_cnn(yolo_root)) {
    return false;
  }
  load_nms_params();
  init_anchor(yolo_root);

  return true;
}
void YoloCameraDetector::init_anchor(const std::string &yolo_root) {
  const auto &model_param = yolo_param_.model_param();
  std::string model_root =
      base::FileUtil::get_absolute_path(yolo_root, model_param.model_name());

  // loading anchors
  std::string anchors_file =
      base::FileUtil::get_absolute_path(model_root, model_param.anchors_file());
  vector<float> anchors;
  yolo::load_anchors(anchors_file, &anchors);
  num_anchors_ = anchors.size() / 2;
  obj_size_ = output_height_ * output_width_ * anchors.size() / 2;
  anchor_.reset(new adu::perception::obstacle::SyncedMemory(anchors.size() *
                                                            sizeof(float)));

  auto anchor_cpu_data = anchor_->mutable_cpu_data();
  memcpy(anchor_cpu_data, anchors.data(), anchors.size() * sizeof(float));
  anchor_->gpu_data();
  // loading types

  std::string types_file =
      base::FileUtil::get_absolute_path(model_root, model_param.types_file());
  yolo::load_types(types_file, &types_);
  res_box_tensor_.reset(new anakin::Tensor<float>());
  res_box_tensor_->Reshape(1, 1, obj_size_, s_box_block_size);

  res_cls_tensor_.reset(new anakin::Tensor<float>());
  res_cls_tensor_->Reshape(1, 1, types_.size(), obj_size_);

  overlapped_.reset(new adu::perception::obstacle::SyncedMemory(
      top_k_ * top_k_ * sizeof(bool)));
  overlapped_->cpu_data();
  overlapped_->gpu_data();
  idx_sm_.reset(
      new adu::perception::obstacle::SyncedMemory(top_k_ * sizeof(int)));
  idx_sm_->cpu_data();
  idx_sm_->gpu_data();
}
void YoloCameraDetector::load_nms_params() {
  auto const &nms_param = yolo_param_.nms_param();
  nms_.sigma = nms_param.sigma();
  nms_.type = nms_param.type();
  nms_.threshold = nms_param.threshold();
  cross_class_merge_threshold_ = nms_param.cross_class_merge_thresh();
  inter_cls_nms_thresh_ = nms_param.inter_cls_nms_thresh();
}
void YoloCameraDetector::load_intrinsic(
    const CameraDetectorInitOptions &options) {
  const auto &model_param = yolo_param_.model_param();

  float offset_ratio = model_param.offset_ratio();
  float cropped_ratio = model_param.cropped_ratio();
  int resized_width = model_param.resized_width();
  int aligned_pixel = model_param.aligned_pixel();
  confidence_threshold_ = model_param.confidence_threshold();
  _min_2d_height = model_param.min_2d_height();
  _min_3d_height = model_param.min_3d_height();
  // inference input shape

  if (options.intrinsic == nullptr) {
    AERROR << "options.intrinsic is nullptr!";
    image_height_ = 1208;
    image_width_ = 1920;
  } else {
    image_height_ = options.intrinsic->get_height();
    image_width_ = options.intrinsic->get_width();
  }

  offset_y_ = int(offset_ratio * image_height_ + .5);
  float roi_ratio = cropped_ratio * image_height_ / image_width_;
  width_ =
      int(resized_width + aligned_pixel / 2) / aligned_pixel * aligned_pixel;
  height_ = int(width_ * roi_ratio + aligned_pixel / 2) / aligned_pixel *
            aligned_pixel;
  AINFO << "image_height=" << image_height_ << ", "
        << "image_width=" << image_width_ << ", "
        << "roi_ratio=" << roi_ratio;
  AINFO << "offset_y=" << offset_y_ << ", height=" << height_
        << ", width=" << width_;
  _min_2d_height /= height_;

  int roi_w = image_width_;
  int roi_h = image_height_ - offset_y_;
  int channel = 3;
  image_data_.reset(
      new SyncedMemory(roi_w * roi_h * channel * sizeof(unsigned char)));
}

bool YoloCameraDetector::init_cnn(const std::string &yolo_root) {
  auto const &net_param = yolo_param_.net_param();
  const auto &model_param = yolo_param_.model_param();
  std::string model_root =
      base::FileUtil::get_absolute_path(yolo_root, model_param.model_name());
  std::string proto_file =
      base::FileUtil::get_absolute_path(model_root, model_param.proto_file());
  std::string weight_file =
      base::FileUtil::get_absolute_path(model_root, model_param.weight_file());
  std::string feature_file =
      base::FileUtil::get_absolute_path(model_root, model_param.feature_file());

  const auto &model_type = model_param.model_type();

  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  // init Net

  input_names.push_back(net_param.input_blob());
  output_names.push_back(net_param.loc_blob());
  output_names.push_back(net_param.obj_blob());
  output_names.push_back(net_param.cls_blob());
  output_names.push_back(net_param.ori_blob());
  output_names.push_back(net_param.dim_blob());
  output_names.push_back(net_param.lof_blob());
  output_names.push_back(net_param.lor_blob());
  output_names.push_back(net_param.seg_blob());

  adu::perception::obstacle::FeatureParam feat_param;
  load_text_proto_message_file(feature_file, feat_param);
  for (auto extractor : feat_param.extractor()) {
    output_names.push_back(extractor.feat_blob());
  }

  // init Net
  AINFO << "model_type=" << model_type;
  switch (model_type) {
    case yolo::ModelType::Caffe:
      cnnadapter_.reset(new CNNCaffe);
      break;
    case yolo::ModelType::Anakin:
      cnnadapter_.reset(new CNNAnakin);

      break;
    case yolo::ModelType::TensorRT:
      cnnadapter_.reset(new CNNTensorRT(false));

      break;
    case yolo::ModelType::TensorRTInt8:
      cnnadapter_.reset(new CNNTensorRT(true));

      break;
  }

  // init feature
  if (!cnnadapter_->init(input_names, output_names, proto_file, weight_file,
                         FLAGS_obs_camera_detector_gpu, model_root)) {
    return false;
  }
  auto obj_blob = cnnadapter_->get_blob_by_name(net_param.obj_blob());
  output_height_ = obj_blob->channels();
  output_width_ = obj_blob->height();

  auto seg_blob = cnnadapter_->get_blob_by_name(net_param.seg_blob());
  if (seg_blob != nullptr) {
    lane_output_height_ = seg_blob->height();
    lane_output_width_ = seg_blob->width();
  }

  std::vector<int> shape = {1, height_, width_, 3};
  if (!cnnadapter_->reshape_input(net_param.input_blob(), shape)) {
    AERROR << "Reshape failed!";
    return false;
  }

  if (feat_param.has_remap_model()) {
    std::string track_model = feat_param.remap_model();
    track_model = base::FileUtil::get_absolute_path(model_root, track_model);
    AINFO << "Using tracking model: " << track_model;
    projector_.reset(new MatrixProjector(track_model));
  } else {
    AINFO << "Using DummyProjector for tracking!";
    projector_.reset(new DummyProjector);
  }
  extractors_.resize(feat_param.extractor_size());
  for (int i = 0; i < feat_param.extractor_size(); ++i) {
    const auto extractor_param = feat_param.extractor(i);
    auto feat_blob = cnnadapter_->get_blob_by_name(extractor_param.feat_blob());
    switch (extractor_param.feat_type()) {
      case ExtractorParam::Reorg:
        extractors_[i].reset(new ReorgFeatureExtractor());
        break;
      case ExtractorParam::ROIPooling:
        extractors_[i].reset(new ROIPoolingFeatureExtractor());
        break;
    }
    extractors_[i]->init(extractor_param, feat_blob, width_, height_);
  }
  return true;
}
bool YoloCameraDetector::multitask(const cv::Mat &frame,
                                   const CameraDetectorOptions &options,
                                   std::vector<VisualObjectPtr> *objects,
                                   cv::Mat *mask) {
  detect(frame, options, objects);
  auto seg_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().seg_blob());
  if (seg_blob == nullptr) {
    return true;
  }

  cv::Mat local_mask(lane_output_height_, lane_output_width_, CV_32FC1);
  memcpy(local_mask.data,
         seg_blob->cpu_data() + lane_output_width_ * lane_output_height_,
         lane_output_width_ * lane_output_height_ * sizeof(float));

  int roi_w = image_width_;
  int roi_h = image_height_ - offset_y_;
  cv::Rect roi(0, offset_y_, roi_w, roi_h);
  if (roi_w == lane_output_width_ && roi_h == lane_output_height_) {
    local_mask.copyTo((*mask)(roi));
  } else {
    cv::resize(local_mask, (*mask)(roi), cv::Size(roi_w, roi_h));
  }
}
bool YoloCameraDetector::detect(const cv::Mat &frame,
                                const CameraDetectorOptions &options,
                                std::vector<VisualObjectPtr> *objects) {
  if (objects == nullptr) {
    return false;
  }

  caffe::Caffe::SetDevice(FLAGS_obs_camera_detector_gpu);
  caffe::Timer pre_time;
  pre_time.Start();
  cv::Mat img;
  int roi_w = frame.cols;
  int roi_h = frame.rows - offset_y_;
  cv::Rect roi(0, offset_y_, roi_w, roi_h);
  auto input_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().input_blob());
  if (roi_w == frame.cols && roi_h == frame.rows) {
    resize(frame, input_blob.get(), image_data_, 0);

  } else {
    resize(frame(roi), input_blob.get(), image_data_, 0);
  }
  pre_time.Stop();
  AINFO << "Pre-processing: " << pre_time.MilliSeconds() << " ms";

  /////////////////////////// detection part ///////////////////////////
  caffe::Timer det_time;
  det_time.Start();
  cnnadapter_->forward();
  AINFO << "Running detection_: " << det_time.MilliSeconds() << " ms";
  caffe::Timer post_time;
  post_time.Start();

  std::vector<VisualObjectPtr> temp_objects;

  get_objects_gpu(&temp_objects);

  AINFO << "object size" << temp_objects.size();
  for (int i = 0; i < temp_objects.size(); ++i) {
    VisualObjectPtr obj = (temp_objects)[i];
    AINFO << "type prob size for object" << i << " is "
          << sizeof(obj->type_probs) << " (" << obj << ")";
    AINFO << "prob: " << obj->type_probs[obj->type];
    AINFO << "object feature size for object" << i << " is "
          << obj->object_feature.size();
    AINFO << "internal type probs size for object" << i << " is "
          << sizeof(obj->internal_type_probs);
  }

  int valid_obj_idx = 0;
  int total_obj_idx = 0;
  auto ori_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().ori_blob());
  auto dim_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().dim_blob());
  if (ori_blob != nullptr && dim_blob != nullptr) {
    while (total_obj_idx < static_cast<int>(temp_objects.size())) {
      const auto &obj = temp_objects[total_obj_idx];
      if ((obj->lower_right[1] - obj->upper_left[1]) >= _min_2d_height &&
          (_min_3d_height <= 0 || obj->height >= _min_3d_height)) {
        objects->push_back(temp_objects[total_obj_idx]);
        ++valid_obj_idx;
      }
      ++total_obj_idx;
    }
    AINFO << valid_obj_idx << " of " << total_obj_idx << " obstacles kept";
    // if (valid_obj_idx > 0) {
    //     objects->resize(valid_obj_idx);
    // }
  }
  for (size_t i = 0; i < temp_objects.size(); ++i) {
    temp_objects[i].reset();
  }
  temp_objects.clear();
  AINFO << "Post-processing: " << post_time.MilliSeconds() << " ms";
  AINFO << "Number of detected obstacles: " << objects->size();

  extract(objects);
  recover_bbox(roi_w, roi_h, offset_y_, objects);

  int det_id = 0;
  for (auto &obj : *objects) {
    projector_->project(obj->object_feature);

    // Assign unique detection box id per box for this frame
    obj->id = det_id;
    AINFO << "obj-" << det_id << ": " << obj->object_feature.size();
    ++det_id;
  }

  return true;
}

std::string YoloCameraDetector::name() const { return "YoloCameraDetector"; }
// Register plugin.
REGISTER_CAMERA_DETECTOR(YoloCameraDetector);
}  // namespace obstacle
}  // namespace perception
}  // namespace apollo
