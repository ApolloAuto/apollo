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

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"

#include "modules/common/util/file.h"
#include "modules/perception/common/mem_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/detector/common/proto/tracking_feature.pb.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/util.h"

DEFINE_string(yolo_config_filename, "config.pt", "Yolo config filename.");

namespace apollo {
namespace perception {

using std::string;
using std::vector;

bool YoloCameraDetector::Init(const CameraDetectorInitOptions &options) {
  ConfigManager *config_manager = ConfigManager::instance();
  string model_name = this->Name();
  const ModelConfig *model_config = NULL;
  if (!config_manager->GetModelConfig(model_name, &model_config)) {
    AERROR << "not found model: " << model_name;
    return false;
  }
  string work_root = config_manager->WorkRoot();

  string yolo_root;
  if (!model_config->GetValue("yolo_root", &yolo_root)) {
    AERROR << "yolo_root not found.";
    return false;
  }
  yolo_root = apollo::common::util::GetAbsolutePath(work_root, yolo_root);

  const string yolo_config = apollo::common::util::GetAbsolutePath(
      yolo_root, FLAGS_yolo_config_filename);
  CHECK(apollo::common::util::GetProtoFromASCIIFile(yolo_config, &yolo_param_));
  load_intrinsic(options);
  if (!init_cnn(yolo_root)) {
    return false;
  }
  load_nms_params();
  init_anchor(yolo_root);

  return true;
}

void YoloCameraDetector::init_anchor(const string &yolo_root) {
  const auto &model_param = yolo_param_.model_param();
  string model_root = apollo::common::util::GetAbsolutePath(
      yolo_root, model_param.model_name());

  // loading anchors
  string anchors_file = apollo::common::util::GetAbsolutePath(
      model_root, model_param.anchors_file());
  vector<float> anchors;
  yolo::load_anchors(anchors_file, &anchors);
  num_anchors_ = anchors.size() / 2;
  obj_size_ = output_height_ * output_width_ * anchors.size() / 2;
  anchor_.reset(new SyncedMemory(anchors.size() * sizeof(float)));

  auto anchor_cpu_data = anchor_->mutable_cpu_data();
  memcpy(anchor_cpu_data, anchors.data(), anchors.size() * sizeof(float));
  anchor_->gpu_data();

  // loading types
  string types_file = apollo::common::util::GetAbsolutePath(
      model_root, model_param.types_file());
  yolo::load_types(types_file, &types_);
  // res_box_tensor_.reset(new anakin::Tensor<float>());
  // res_box_tensor_->Reshape(1, 1, obj_size_, s_box_block_size);

  // res_cls_tensor_.reset(new anakin::Tensor<float>());
  // res_cls_tensor_->Reshape(1, 1, types_.size(), obj_size_);

  overlapped_.reset(new SyncedMemory(top_k_ * top_k_ * sizeof(bool)));
  overlapped_->cpu_data();
  overlapped_->gpu_data();
  idx_sm_.reset(new SyncedMemory(top_k_ * sizeof(int)));
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

  offset_y_ = static_cast<int>(offset_ratio * image_height_ + .5);
  float roi_ratio = cropped_ratio * image_height_ / image_width_;
  width_ = static_cast<int>(resized_width + aligned_pixel / 2) / aligned_pixel *
           aligned_pixel;
  height_ = static_cast<int>(width_ * roi_ratio + aligned_pixel / 2) /
            aligned_pixel * aligned_pixel;
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

bool YoloCameraDetector::init_cnn(const string &yolo_root) {
  auto const &net_param = yolo_param_.net_param();
  const auto &model_param = yolo_param_.model_param();
  string model_root = apollo::common::util::GetAbsolutePath(
      yolo_root, model_param.model_name());
  string proto_file = apollo::common::util::GetAbsolutePath(
      model_root, model_param.proto_file());
  string weight_file = apollo::common::util::GetAbsolutePath(
      model_root, model_param.weight_file());
  string feature_file = apollo::common::util::GetAbsolutePath(
      model_root, model_param.feature_file());

  const auto &model_type = model_param.model_type();

  vector<string> input_names;
  vector<string> output_names;
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

  FeatureParam feat_param;
  CHECK(apollo::common::util::GetProtoFromASCIIFile(feature_file, &feat_param));
  for (auto extractor : feat_param.extractor()) {
    output_names.push_back(extractor.feat_blob());
  }

  // init Net
  AINFO << "model_type=" << model_type;
  switch (model_type) {
    case obstacle::yolo::ModelType::Caffe:
      cnnadapter_.reset(new CNNCaffe);
      break;
    default:
      AFATAL << "unknown model type.";
      return false;
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

  vector<int> shape = {1, height_, width_, 3};
  if (!cnnadapter_->reshape_input(net_param.input_blob(), shape)) {
    AERROR << "Reshape failed!";
    return false;
  }

  if (feat_param.has_remap_model()) {
    string track_model = feat_param.remap_model();
    track_model =
        apollo::common::util::GetAbsolutePath(model_root, track_model);
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

bool YoloCameraDetector::Multitask(const cv::Mat &frame,
                                   const CameraDetectorOptions &options,
                                   vector<VisualObjectPtr> *objects,
                                   cv::Mat *mask) {
  Detect(frame, options, objects);
  auto seg_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().seg_blob());
  if (seg_blob == nullptr) {
    return false;
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

  return true;
}

bool YoloCameraDetector::Detect(const cv::Mat &frame,
                                const CameraDetectorOptions &options,
                                vector<VisualObjectPtr> *objects) {
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

  vector<VisualObjectPtr> temp_objects;

  get_objects_gpu(&temp_objects);

  AINFO << "object size" << temp_objects.size();
  for (int i = 0; i < static_cast<int>(temp_objects.size()); ++i) {
    VisualObjectPtr obj = (temp_objects)[i];
    AINFO << "type prob size for object" << i << " is "
          << sizeof(obj->type_probs) << " (" << obj << ")";
    AINFO << "prob: " << obj->type_probs[static_cast<int>(obj->type)];
    AINFO << "object feature size for object" << i << " is "
          << obj->object_feature.size();
    AINFO << "internal type probs size for object" << i << " is "
          << sizeof(obj->internal_type_probs);
  }

  auto ori_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().ori_blob());
  auto dim_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().dim_blob());
  if (ori_blob != nullptr && dim_blob != nullptr) {
    int valid_obj_idx = 0;
    int total_obj_idx = 0;
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

  Extract(objects);
  recover_bbox(roi_w, roi_h, offset_y_, objects);

  int det_id = 0;
  for (auto &obj : *objects) {
    projector_->project(&(obj->object_feature));

    // Assign unique detection box id per box for this frame
    obj->id = det_id;
    AINFO << "obj-" << det_id << ": " << obj->object_feature.size();
    ++det_id;
  }

  return true;
}

string YoloCameraDetector::Name() const { return "YoloCameraDetector"; }

bool YoloCameraDetector::get_objects_gpu(
    std::vector<VisualObjectPtr> *objects) {
  AFATAL << "Not implemented, yet!";
  return false;
}

}  // namespace perception
}  // namespace apollo
