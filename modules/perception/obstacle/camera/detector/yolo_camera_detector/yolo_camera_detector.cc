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

#include <cmath>
#include <unordered_map>
#include <utility>

#include "modules/perception/obstacle/camera/detector/common/proto/tracking_feature.pb.h"

#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/util.h"

namespace apollo {
namespace perception {

using apollo::common::util::GetProtoFromFile;
using std::string;
using std::unordered_map;
using std::vector;

bool YoloCameraDetector::Init(const CameraDetectorInitOptions &options) {
  // load yolo camera detector config file to proto
  CHECK(GetProtoFromFile(FLAGS_yolo_camera_detector_config, &config_));

  const string &yolo_root = config_.yolo_root();
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
  anchor_.reset(new caffe::SyncedMemory(anchors.size() * sizeof(float)));

  auto anchor_cpu_data = anchor_->mutable_cpu_data();
  memcpy(anchor_cpu_data, anchors.data(), anchors.size() * sizeof(float));

  // loading types
  string types_file = apollo::common::util::GetAbsolutePath(
      model_root, model_param.types_file());
  yolo::load_types(types_file, &types_);

  res_box_tensor_.reset(
      new caffe::SyncedMemory(obj_size_ * s_box_block_size * sizeof(float)));
  res_box_tensor_->cpu_data();
  res_box_tensor_->gpu_data();

  res_cls_tensor_.reset(
      new caffe::SyncedMemory(types_.size() * obj_size_ * sizeof(float)));
  res_cls_tensor_->cpu_data();
  res_cls_tensor_->gpu_data();

  overlapped_.reset(new caffe::SyncedMemory(top_k_ * top_k_ * sizeof(bool)));
  overlapped_->cpu_data();
  overlapped_->gpu_data();

  idx_sm_.reset(new caffe::SyncedMemory(top_k_ * sizeof(int)));
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
  min_2d_height_ = model_param.min_2d_height();
  min_3d_height_ = model_param.min_3d_height();

  // inference input shape
  if (options.intrinsic == nullptr) {
    AWARN << "YoloCameraDetector options.intrinsic is nullptr. Use default";
    image_height_ = 1080;
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
  ADEBUG << "image_height=" << image_height_ << ", "
         << "image_width=" << image_width_ << ", "
         << "roi_ratio=" << roi_ratio;
  ADEBUG << "offset_y=" << offset_y_ << ", height=" << height_
         << ", width=" << width_;

  min_2d_height_ /= height_;

  int roi_w = image_width_;
  int roi_h = image_height_ - offset_y_;

  ADEBUG << "roi_w=" << roi_w << ", "
         << "roi_h=" << roi_h;

  int channel = 3;
  image_data_.reset(
      new caffe::SyncedMemory(roi_w * roi_h * channel * sizeof(unsigned char)));
}

bool YoloCameraDetector::init_cnn(const string &yolo_root) {
  ADEBUG << "yolo_root: " << yolo_root;

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

  ADEBUG << " proto_file: " << proto_file;
  ADEBUG << " weight_file: " << weight_file;
  ADEBUG << " model_root: " << model_root;
  ADEBUG << " feature_file: " << feature_file;

  const auto &model_type = model_param.model_type();

  vector<string> input_names;
  vector<string> output_names;
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
  ADEBUG << "model_type=" << model_type;
  switch (model_type) {
    case obstacle::yolo::ModelType::Caffe:
      cnnadapter_.reset(new CNNCaffe);
      break;
    default:
      AERROR << "unknown model type.";
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
    ADEBUG << "Using tracking model: " << track_model;
    projector_.reset(new MatrixProjector(track_model));
  } else {
    ADEBUG << "Using DummyProjector for tracking!";
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

bool YoloCameraDetector::Multitask(
    const cv::Mat &frame, const CameraDetectorOptions &options,
    vector<std::shared_ptr<VisualObject>> *objects, cv::Mat *mask) {
  if (objects == nullptr) {
    AERROR << "'objects' is a null pointer.";
    return false;
  }
  if (mask == nullptr) {
    AERROR << "'mask' is a null pointer.";
    return false;
  }

  Detect(frame, options, objects);

  auto seg_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().seg_blob());
  if (seg_blob == nullptr) {
    AERROR << "'seg_blob' is a null pointer.";
    return false;
  }

  *mask = cv::Mat(lane_output_height_, lane_output_width_, CV_32FC1);
  memcpy(mask->data,
         seg_blob->cpu_data() + lane_output_width_ * lane_output_height_,
         lane_output_width_ * lane_output_height_ * sizeof(float));

  return true;
}

bool YoloCameraDetector::Detect(
    const cv::Mat &frame, const CameraDetectorOptions &options,
    vector<std::shared_ptr<VisualObject>> *objects) {
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
  AINFO << "Running detection: " << det_time.MilliSeconds() << " ms";
  caffe::Timer post_time;
  post_time.Start();

  vector<std::shared_ptr<VisualObject>> temp_objects;

  if (FLAGS_obs_camera_detector_gpu >= 0) {
    ADEBUG << "Get objects by GPU";
    get_objects_gpu(&temp_objects);
  } else {
    get_objects_cpu(&temp_objects);
  }

  ADEBUG << "object size = " << temp_objects.size();
  for (int i = 0; i < static_cast<int>(temp_objects.size()); ++i) {
    std::shared_ptr<VisualObject> obj = (temp_objects)[i];
    ADEBUG << "type prob size for object" << i << " is "
           << sizeof(obj->type_probs) << " (" << obj << ")";
    ADEBUG << "prob: " << obj->type_probs[static_cast<int>(obj->type)];
    ADEBUG << "object feature size for object" << i << " is "
           << obj->object_feature.size();
    ADEBUG << "internal type probs size for object" << i << " is "
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
      if ((obj->lower_right[1] - obj->upper_left[1]) >= min_2d_height_ &&
          (min_3d_height_ <= 0 || obj->height >= min_3d_height_)) {
        objects->push_back(temp_objects[total_obj_idx]);
        ++valid_obj_idx;
      }
      ++total_obj_idx;
    }
    ADEBUG << valid_obj_idx << " of " << total_obj_idx << " obstacles kept";
  }
  for (size_t i = 0; i < temp_objects.size(); ++i) {
    temp_objects[i].reset();
  }
  temp_objects.clear();
  AINFO << "Post-processing: " << post_time.MilliSeconds() << " ms";
  AINFO << "Number of detected obstacles: " << objects->size();

  Extract(objects);
  yolo::recover_bbox(roi_w, roi_h, offset_y_, objects);

  int det_id = 0;
  for (auto &obj : *objects) {
    projector_->project(&(obj->object_feature));

    // Assign detection id per box and score
    obj->id = det_id;
    for (auto prob : obj->type_probs) {
      obj->score = std::max(obj->score, prob);
    }

    ADEBUG << "obj-" << det_id << ": " << obj->object_feature.size();
    det_id++;
  }

  return true;
}

string YoloCameraDetector::Name() const { return "YoloCameraDetector"; }

bool YoloCameraDetector::get_objects_cpu(
    std::vector<std::shared_ptr<VisualObject>> *objects) {
  int num_classes = types_.size();
  auto loc_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().loc_blob());
  auto obj_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().obj_blob());
  auto cls_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().cls_blob());
  auto dim_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().dim_blob());
  auto ori_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().ori_blob());
  auto lof_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().lof_blob());
  auto lor_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().lor_blob());

  int batch = obj_blob->num();
  int height = obj_blob->channels();
  int width = obj_blob->height();
  CHECK_EQ(batch, 1) << "batch size should be 1!";
  const float *loc_data = loc_blob->cpu_data();
  const float *obj_data = obj_blob->cpu_data();
  const float *cls_data = cls_blob->cpu_data();

  bool with_lof = lof_blob != nullptr;
  bool with_lor = lor_blob != nullptr;
  bool with_ori = ori_blob != nullptr;
  bool with_dim = dim_blob != nullptr;

  const float *ori_data = with_ori ? ori_blob->cpu_data() : nullptr;
  const float *dim_data = with_dim ? dim_blob->cpu_data() : nullptr;
  const float *lof_data = with_lof ? lof_blob->cpu_data() : nullptr;
  const float *lor_data = with_lor ? lor_blob->cpu_data() : nullptr;
  if (res_box_tensor_ == nullptr || res_cls_tensor_ == nullptr ||
      overlapped_ == nullptr) {
    return false;
  }

  for (int i = 0; i < obj_size_; ++i) {
    get_object_helper(i, loc_data, obj_data, cls_data, ori_data, dim_data,
                      lof_data, lor_data, width, height, num_classes, with_ori,
                      with_dim, with_lof, with_lor);
  }

  const float *cpu_cls_data =
      static_cast<const float *>(res_cls_tensor_->cpu_data());

  unordered_map<int, vector<int>> indices;
  unordered_map<int, vector<float>> conf_scores;
  int num_kept = 0;
  for (int k = 0; k < num_classes; k++) {
    apply_nms_gpu(static_cast<const float *>(res_box_tensor_->gpu_data()),
                  cpu_cls_data + k * obj_size_, obj_size_,
                  confidence_threshold_, top_k_, nms_.threshold,
                  &(indices[static_cast<int>(types_[k])]), overlapped_,
                  idx_sm_);
    num_kept += indices[static_cast<int>(types_[k])].size();
    vector<float> conf_score(cpu_cls_data + k * obj_size_,
                             cpu_cls_data + (k + 1) * obj_size_);
    conf_scores.insert(std::make_pair(static_cast<int>(types_[k]), conf_score));
  }
  if (num_kept == 0) {
    ADEBUG << "Couldn't find any detections";
    return true;
  }

  objects->clear();
  objects->reserve(num_kept);
  const float *cpu_box_data =
      static_cast<const float *>(res_box_tensor_->cpu_data());

  for (auto it = indices.begin(); it != indices.end(); ++it) {
    int label = it->first;
    if (conf_scores.find(label) == conf_scores.end()) {
      // Something bad happened if there are no predictions for current label.
      AERROR << "Could not find confidence predictions for " << label;
      continue;
    }

    const vector<float> &scores = conf_scores.find(label)->second;
    vector<int> &indice = it->second;
    for (int j = 0; j < static_cast<int>(indice.size()); ++j) {
      int idx = indice[j];
      const float *bbox = cpu_box_data + idx * s_box_block_size;
      if (scores[idx] < confidence_threshold_) {
        continue;
      }

      std::shared_ptr<VisualObject> obj(new VisualObject);
      obj->type = static_cast<ObjectType>(label);
      obj->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),
                             0.0f);
      for (int k = 0; k < num_classes; ++k) {
        int type_k = static_cast<int>(types_[k]);
        obj->type_probs[type_k] = conf_scores[type_k][idx];
      }
      obj->upper_left[0] = bbox[0];
      obj->upper_left[1] = bbox[1];
      obj->lower_right[0] = bbox[2];
      obj->lower_right[1] = bbox[3];

      if (with_ori) {
        obj->alpha = bbox[4];
      }

      if (with_dim) {
        obj->height = bbox[5];
        obj->width = bbox[6];
        obj->length = bbox[7];
      }

      if (with_lof) {
        obj->front_upper_left[0] = bbox[8];
        obj->front_upper_left[1] = bbox[9];
        obj->front_lower_right[0] = bbox[10];
        obj->front_lower_right[1] = bbox[11];
      }

      if (with_lor) {
        obj->back_upper_left[0] = bbox[12];
        obj->back_upper_left[1] = bbox[13];
        obj->back_lower_right[0] = bbox[14];
        obj->back_lower_right[1] = bbox[15];
      }

      obj->object_feature.clear();
      objects->push_back(obj);
    }
  }

  return true;
}
bool YoloCameraDetector::get_objects_gpu(
    std::vector<std::shared_ptr<VisualObject>> *objects) {
  auto loc_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().loc_blob());
  auto obj_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().obj_blob());
  auto cls_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().cls_blob());
  auto dim_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().dim_blob());
  auto ori_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().ori_blob());
  auto lof_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().lof_blob());
  auto lor_blob =
      cnnadapter_->get_blob_by_name(yolo_param_.net_param().lor_blob());
  const float *anchor_data = static_cast<const float *>(anchor_->gpu_data());
  int num_classes = types_.size();
  int obj_batch = obj_blob->num();
  int obj_height = obj_blob->channels();
  int obj_width = obj_blob->height();
  CHECK_EQ(obj_batch, 1) << "batch size should be 1!";
  bool with_lof = lof_blob != nullptr;
  bool with_lor = lor_blob != nullptr;
  bool with_ori = ori_blob != nullptr;
  bool with_dim = dim_blob != nullptr;
  const float *ori_data = with_ori ? ori_blob->gpu_data() : nullptr;
  const float *dim_data = with_dim ? dim_blob->gpu_data() : nullptr;
  const float *lof_data = with_lof ? lof_blob->gpu_data() : nullptr;
  const float *lor_data = with_lor ? lor_blob->gpu_data() : nullptr;
  if (res_box_tensor_ == nullptr || res_cls_tensor_ == nullptr ||
      overlapped_ == nullptr) {
    return false;
  }
  GetObjectsGPU(obj_size_, (const float *)loc_blob->gpu_data(),
                (const float *)obj_blob->gpu_data(),
                (const float *)cls_blob->gpu_data(), ori_data, dim_data,
                lof_data, lor_data, anchor_data, obj_width, obj_height,
                num_anchors_, num_classes, confidence_threshold_, with_ori,
                with_dim, with_lof, with_lor,
                static_cast<float *>(res_box_tensor_->mutable_gpu_data()),
                static_cast<float *>(res_cls_tensor_->mutable_gpu_data()),
                s_box_block_size);
  const float *cpu_cls_data =
      static_cast<const float *>(res_cls_tensor_->cpu_data());

  unordered_map<int, vector<int>> indices;
  unordered_map<int, vector<float>> conf_scores;
  int num_kept = 0;
  for (int k = 0; k < num_classes; k++) {
    apply_nms_gpu(static_cast<const float *>(res_box_tensor_->gpu_data()),
                  cpu_cls_data + k * obj_size_, obj_size_,
                  confidence_threshold_, top_k_, nms_.threshold,
                  &(indices[static_cast<int>(types_[k])]), overlapped_,
                  idx_sm_);
    num_kept += indices[static_cast<int>(types_[k])].size();
    vector<float> conf_score(cpu_cls_data + k * obj_size_,
                             cpu_cls_data + (k + 1) * obj_size_);
    conf_scores.insert(std::make_pair(static_cast<int>(types_[k]), conf_score));
  }
  if (num_kept == 0) {
    AINFO << "Couldn't find any detections";
    return true;
  }

  objects->clear();
  objects->reserve(num_kept);
  const float *cpu_box_data =
      static_cast<const float *>(res_box_tensor_->cpu_data());

  for (auto it = indices.begin(); it != indices.end(); ++it) {
    int label = it->first;
    if (conf_scores.find(label) == conf_scores.end()) {
      // Something bad happened if there are no predictions for current label.
      AERROR << "Could not find confidence predictions for " << label;
      continue;
    }

    const vector<float> &scores = conf_scores.find(label)->second;
    vector<int> &indice = it->second;
    for (int j = 0; j < static_cast<int>(indice.size()); ++j) {
      int idx = indice[j];
      const float *bbox = cpu_box_data + idx * s_box_block_size;
      if (scores[idx] < confidence_threshold_) {
        continue;
      }

      std::shared_ptr<VisualObject> obj(new VisualObject);
      obj->type = static_cast<ObjectType>(label);
      obj->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),
                             0.0f);
      for (int k = 0; k < num_classes; ++k) {
        int type_k = static_cast<int>(types_[k]);
        obj->type_probs[type_k] = conf_scores[type_k][idx];
      }
      obj->upper_left[0] = bbox[0];
      obj->upper_left[1] = bbox[1];
      obj->lower_right[0] = bbox[2];
      obj->lower_right[1] = bbox[3];

      if (with_ori) {
        obj->alpha = bbox[4];
      }

      if (with_dim) {
        obj->height = bbox[5];
        obj->width = bbox[6];
        obj->length = bbox[7];
      }

      if (with_lof) {
        obj->front_upper_left[0] = bbox[8];
        obj->front_upper_left[1] = bbox[9];
        obj->front_lower_right[0] = bbox[10];
        obj->front_lower_right[1] = bbox[11];
      }

      if (with_lor) {
        obj->back_upper_left[0] = bbox[12];
        obj->back_upper_left[1] = bbox[13];
        obj->back_lower_right[0] = bbox[14];
        obj->back_lower_right[1] = bbox[15];
      }

      obj->object_feature.clear();
      objects->push_back(obj);
    }
  }
  return true;
}

void YoloCameraDetector::get_object_helper(
    const int i, const float *loc_data, const float *obj_data,
    const float *cls_data, const float *ori_data, const float *dim_data,
    const float *lof_data, const float *lor_data, const int width,
    const int height, const int num_classes, const bool with_ori,
    const bool with_dim, const bool with_lof, const bool with_lor) {
  const float *anchor_data = static_cast<const float *>(anchor_->cpu_data());
  float *res_box_data =
      static_cast<float *>(res_box_tensor_->mutable_cpu_data());
  float *res_cls_data =
      static_cast<float *>(res_cls_tensor_->mutable_cpu_data());

  int box_block = s_box_block_size;

  int idx = i;
  int c = idx % num_anchors_;
  idx = idx / num_anchors_;
  int w = idx % width;
  idx = idx / width;
  int h = idx;
  int offset_loc = ((h * width + w) * num_anchors_ + c) * 4;
  int offset_obj = (h * width + w) * num_anchors_ + c;
  int offset_cls = ((h * width + w) * num_anchors_ + c) * num_classes;
  float scale = obj_data[offset_obj];

  float cx = (w + yolo::sigmoid(loc_data[offset_loc + 0])) / width;
  float cy = (h + yolo::sigmoid(loc_data[offset_loc + 1])) / height;
  float hw =
      std::exp(loc_data[offset_loc + 2]) * anchor_data[2 * c] / width * 0.5;
  float hh = std::exp(loc_data[offset_loc + 3]) * anchor_data[2 * c + 1] /
             height * 0.5;

  for (int k = 0; k < num_classes; ++k) {
    float prob = (cls_data[offset_cls + k] * scale > confidence_threshold_
                      ? cls_data[offset_cls + k] * scale
                      : 0);
    res_cls_data[k * width * height * num_anchors_ + i] = prob;
  }
  res_box_data[i * box_block + 0] = cx - hw;
  res_box_data[i * box_block + 1] = cy - hh;
  res_box_data[i * box_block + 2] = cx + hw;
  res_box_data[i * box_block + 3] = cy + hh;

  if (with_ori) {
    int offset_ori = ((h * width + w) * num_anchors_ + c) * 2;
    res_box_data[i * box_block + 4] =
        std::atan2(ori_data[offset_ori + 1], ori_data[offset_ori]);
  }

  if (with_dim) {
    int offset_dim = ((h * width + w) * num_anchors_ + c) * 3;
    res_box_data[i * box_block + 5] = dim_data[offset_dim + 0];
    res_box_data[i * box_block + 6] = dim_data[offset_dim + 1];
    res_box_data[i * box_block + 7] = dim_data[offset_dim + 2];
  }

  float *dst_ptr = nullptr;
  const float *src_ptr = nullptr;
  if (with_lof) {
    int offset_lof = ((h * width + w) * num_anchors_ + c) * 4;
    dst_ptr = res_box_data + i * box_block + 8;
    src_ptr = lof_data + offset_lof;
    float sb_x = src_ptr[0] * hw * 2.0f + cx;
    float sb_y = src_ptr[1] * hh * 2.0f + cy;
    float sb_hw = std::exp(src_ptr[2]) * hw;
    float sb_hh = std::exp(src_ptr[3]) * hh;
    dst_ptr[0] = sb_x - sb_hw;
    dst_ptr[1] = sb_y - sb_hh;
    dst_ptr[2] = sb_x + sb_hw;
    dst_ptr[3] = sb_y + sb_hh;
  }

  if (with_lor) {
    int offset_lor = ((h * width + w) * num_anchors_ + c) * 4;
    dst_ptr = res_box_data + i * box_block + 12;
    src_ptr = lor_data + offset_lor;
    float sb_x = src_ptr[0] * hw * 2.0f + cx;
    float sb_y = src_ptr[1] * hh * 2.0f + cy;
    float sb_hw = std::exp(src_ptr[2]) * hw;
    float sb_hh = std::exp(src_ptr[3]) * hh;
    dst_ptr[0] = sb_x - sb_hw;
    dst_ptr[1] = sb_y - sb_hh;
    dst_ptr[2] = sb_x + sb_hw;
    dst_ptr[3] = sb_y + sb_hh;
  }
}

}  // namespace perception
}  // namespace apollo
