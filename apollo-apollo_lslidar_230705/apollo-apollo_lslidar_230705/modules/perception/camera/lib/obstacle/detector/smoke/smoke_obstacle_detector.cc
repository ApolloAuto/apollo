/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera/lib/obstacle/detector/smoke/smoke_obstacle_detector.h"

#include <opencv2/opencv.hpp>
#include <boost/algorithm/string.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/base/common.h"
#include "modules/perception/camera/common/timer.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/inference/inference_factory.h"
#include "modules/perception/inference/utils/resize.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

void SmokeObstacleDetector::LoadInputShape(
    const smoke::ModelParam &model_param) {
  float offset_ratio = model_param.offset_ratio();
  float cropped_ratio = model_param.cropped_ratio();
  int resized_width = model_param.resized_width();
  int aligned_pixel = model_param.aligned_pixel();
  // inference input shape
  int image_height = static_cast<int>(base_camera_model_->get_height());
  int image_width = static_cast<int>(base_camera_model_->get_width());

  offset_y_ =
      static_cast<int>(offset_ratio * static_cast<float>(image_height) + .5f);
  float roi_ratio = cropped_ratio * static_cast<float>(image_height) /
                    static_cast<float>(image_width);
  width_ = static_cast<int>(resized_width + aligned_pixel / 2) / aligned_pixel *
           aligned_pixel;
  height_ = static_cast<int>(static_cast<float>(width_) * roi_ratio +
                             static_cast<float>(aligned_pixel) / 2.0f) /
            aligned_pixel * aligned_pixel;

  AINFO << "image_height=" << image_height << ", "
        << "image_width=" << image_width << ", "
        << "roi_ratio=" << roi_ratio;
  AINFO << "offset_y=" << offset_y_ << ", height=" << height_
        << ", width=" << width_;
}

void SmokeObstacleDetector::LoadParam(const smoke::SmokeParam &smoke_param) {
  const auto &model_param = smoke_param.model_param();
  confidence_threshold_ = model_param.confidence_threshold();
  light_vis_conf_threshold_ = model_param.light_vis_conf_threshold();
  light_swt_conf_threshold_ = model_param.light_swt_conf_threshold();
  min_dims_.min_2d_height = model_param.min_2d_height();
  min_dims_.min_3d_height = model_param.min_3d_height();
  min_dims_.min_3d_width = model_param.min_3d_width();
  min_dims_.min_3d_length = model_param.min_3d_length();
  ori_cycle_ = model_param.ori_cycle();

  border_ratio_ = model_param.border_ratio();

  // init NMS
  auto const &nms_param = smoke_param.nms_param();
  nms_.sigma = nms_param.sigma();
  nms_.type = nms_param.type();
  nms_.threshold = nms_param.threshold();
  nms_.inter_cls_nms_thresh = nms_param.inter_cls_nms_thresh();
  nms_.inter_cls_conf_thresh = nms_param.inter_cls_conf_thresh();
}

bool SmokeObstacleDetector::InitNet(const smoke::SmokeParam &smoke_param,
                                    const std::string &model_root) {
  const auto &model_param = smoke_param.model_param();

  std::string proto_file =
      GetAbsolutePath(model_root, model_param.proto_file());
  std::string weight_file =
      GetAbsolutePath(model_root, model_param.weight_file());
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  // init Net
  auto const &net_param = smoke_param.net_param();
  input_names.push_back(net_param.input_data_blob());
  input_names.push_back(net_param.input_instric_blob());
  input_names.push_back(net_param.input_ratio_blob());
  output_names.push_back(net_param.det1_loc_blob());
  output_names.push_back(net_param.feat_blob());

  // init Net
  const auto &model_type = model_param.model_type();
  AINFO << "model_type=" << model_type;
  inference_.reset(inference::CreateInferenceByName(model_type, proto_file,
                                                    weight_file, output_names,
                                                    input_names, model_root));
  if (nullptr == inference_.get()) {
    AERROR << "Failed to init CNNAdapter";
    return false;
  }
  inference_->set_gpu_id(gpu_id_);
  std::vector<int> shape_input = {1, height_, width_, 3};
  std::vector<int> shape_param = {1, 3, 3};
  std::vector<int> shape_ratio = {1, 2};
  std::vector<int> shape_res = {1, 1, 50, 14};
  std::vector<int> shape_feat = {1, 64, 160, 240};
  std::map<std::string, std::vector<int>> shape_map;
  shape_map.emplace(
      (std::pair<std::string, std::vector<int>>(input_names[0], shape_input)));
  shape_map.emplace(
      (std::pair<std::string, std::vector<int>>(input_names[1], shape_param)));
  shape_map.emplace(
      (std::pair<std::string, std::vector<int>>(input_names[2], shape_ratio)));
  shape_map.emplace(
      (std::pair<std::string, std::vector<int>>(output_names[0], shape_res)));
  shape_map.emplace(
      (std::pair<std::string, std::vector<int>>(output_names[1], shape_feat)));
  if (!inference_->Init(shape_map)) {
    return false;
  }
  // inference_->Infer();
  return true;
}

void SmokeObstacleDetector::InitSmokeBlob(
    const smoke::NetworkParam &net_param) {
  auto obj_blob_scale1 = inference_->get_blob(net_param.det1_obj_blob());
  overlapped_.reset(
      new base::Blob<bool>(std::vector<int>{obj_k_, obj_k_}, true));
  overlapped_->cpu_data();
  overlapped_->gpu_data();
  idx_sm_.reset(new base::Blob<int>(std::vector<int>{obj_k_}, true));
  image_.reset(new base::Image8U(height_, width_, base::Color::RGB));

  smoke_blobs_.det1_loc_blob =
      inference_->get_blob(smoke_param_.net_param().det1_loc_blob());
}

bool SmokeObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  gpu_id_ = options.gpu_id;
  BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
  BASE_GPU_CHECK(cudaStreamCreate(&stream_));

  base_camera_model_ = options.base_camera_model;
  ACHECK(base_camera_model_ != nullptr) << "base_camera_model is nullptr!";
  std::string config_path =
      GetAbsolutePath(options.root_dir, options.conf_file);
  if (!cyber::common::GetProtoFromFile(config_path, &smoke_param_)) {
    AERROR << "read proto_config fail";
    return false;
  }
  const auto &model_param = smoke_param_.model_param();
  std::string model_root =
      GetAbsolutePath(options.root_dir, model_param.model_name());
  std::string anchors_file =
      GetAbsolutePath(model_root, model_param.anchors_file());
  std::string types_file =
      GetAbsolutePath(model_root, model_param.types_file());
  std::string expand_file =
      GetAbsolutePath(model_root, model_param.expand_file());
  LoadInputShape(model_param);
  LoadParam(smoke_param_);
  min_dims_.min_2d_height /= static_cast<float>(height_);

  if (!LoadAnchors(anchors_file, &anchors_)) {
    return false;
  }
  if (!LoadTypes(types_file, &types_)) {
    return false;
  }
  if (!LoadExpand(expand_file, &expands_)) {
    return false;
  }
  ACHECK(expands_.size() == types_.size());
  if (!InitNet(smoke_param_, model_root)) {
    return false;
  }
  InitSmokeBlob(smoke_param_.net_param());
  if (!InitFeatureExtractor(model_root)) {
    return false;
  }
  return true;
}

bool SmokeObstacleDetector::Init(const StageConfig &stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  ACHECK(stage_config.has_camera_detector_config());
  auto smoke_obstacle_detection_config_ =
      stage_config.camera_detector_config();

  gpu_id_ = smoke_obstacle_detection_config_.gpu_id();
  BASE_GPU_CHECK(cudaSetDevice(gpu_id_));
  BASE_GPU_CHECK(cudaStreamCreate(&stream_));

  std::string camera_name =
          smoke_obstacle_detection_config_.camera_name();
  boost::algorithm::split(camera_names_, camera_name,
                              boost::algorithm::is_any_of(","));
  base_camera_model_ =
      common::SensorManager::Instance()->GetUndistortCameraModel(
          camera_names_[0]);
  ACHECK(base_camera_model_ != nullptr) << "base_camera_model is nullptr!";

  std::string config_path =
      GetAbsolutePath(smoke_obstacle_detection_config_.root_dir(),
                      smoke_obstacle_detection_config_.conf_file());
  if (!cyber::common::GetProtoFromFile(config_path, &smoke_param_)) {
    AERROR << "read proto_config fail";
    return false;
  }
  const auto &model_param = smoke_param_.model_param();
  std::string model_root = GetAbsolutePath(
      smoke_obstacle_detection_config_.root_dir(), model_param.model_name());
  std::string anchors_file =
      GetAbsolutePath(model_root, model_param.anchors_file());
  std::string types_file =
      GetAbsolutePath(model_root, model_param.types_file());
  std::string expand_file =
      GetAbsolutePath(model_root, model_param.expand_file());
  LoadInputShape(model_param);
  LoadParam(smoke_param_);
  min_dims_.min_2d_height /= static_cast<float>(height_);

  if (!LoadAnchors(anchors_file, &anchors_)) {
    return false;
  }
  if (!LoadTypes(types_file, &types_)) {
    return false;
  }
  if (!LoadExpand(expand_file, &expands_)) {
    return false;
  }
  ACHECK(expands_.size() == types_.size());
  if (!InitNet(smoke_param_, model_root)) {
    return false;
  }
  InitSmokeBlob(smoke_param_.net_param());
  if (!InitFeatureExtractor(model_root)) {
    return false;
  }
  return true;
}

bool SmokeObstacleDetector::InitFeatureExtractor(const std::string &root_dir) {
  FeatureExtractorInitOptions feature_options;
  feature_options.conf_file = smoke_param_.model_param().feature_file();
  feature_options.root_dir = root_dir;
  feature_options.gpu_id = gpu_id_;
  auto feat_blob_name = smoke_param_.net_param().feat_blob();
  feature_options.feat_blob = inference_->get_blob(feat_blob_name);
  feature_options.input_height = height_;
  feature_options.input_width = width_;
  feature_extractor_.reset(BaseFeatureExtractorRegisterer::GetInstanceByName(
      "TrackingFeatureExtractor"));
  if (!feature_extractor_->Init(feature_options)) {
    return false;
  }
  return true;
}

bool SmokeObstacleDetector::Preprocessor(const base::Image8U* image,
    std::shared_ptr<base::Blob<float>> input_blob) {
  cv::Mat img = cv::Mat(image->rows(), image->cols(), CV_8UC3);
  memcpy(img.data, image->cpu_data(),
      image->rows() * image->cols() * image->channels() * sizeof(uint8_t));

  // resize
  cv::resize(img, img, cv::Size(width_, height_));

  // mean and std
  img.convertTo(img, CV_32F, 1.0 / 255, 0);
  std::vector<float> mean_values{0, 0, 0};
  std::vector<float> std_values{0.229, 0.224, 0.225};

  std::vector<cv::Mat> rgbChannels(3);
  cv::split(img, rgbChannels);
  for (int i = 0; i < 3; ++i) {
    rgbChannels[i].convertTo(rgbChannels[i], CV_32FC1, 1 / std_values[i],
                             (0.0 - mean_values[i]) / std_values[i]);
  }
  cv::merge(rgbChannels, img);

  // from hwc to chw
  int rows = img.rows;
  int cols = img.cols;
  int chs = img.channels();

  // fill input_blob
  input_blob->Reshape({1, chs, rows, cols});
  float* input_data = input_blob->mutable_cpu_data();
  for (int i = 0; i < chs; ++i) {
    cv::extractChannel(
        img, cv::Mat(rows, cols, CV_32FC1, input_data + i * rows * cols), i);
  }
  return true;
}

bool SmokeObstacleDetector::Detect(const ObstacleDetectorOptions &options,
                                   CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }

  Timer timer;
  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to " << gpu_id_;
    return false;
  }
  const auto &camera_k_matrix = frame->camera_k_matrix.inverse();
  auto const &net_param = smoke_param_.net_param();
  auto input_blob = inference_->get_blob(net_param.input_data_blob());
  auto input_K_blob = inference_->get_blob(net_param.input_instric_blob());
  auto input_ratio_blob = inference_->get_blob(net_param.input_ratio_blob());

  float *ratio_data = input_ratio_blob->mutable_cpu_data();
  float *K_data = input_K_blob->mutable_cpu_data();
  for (size_t i = 0; i < 3; i++) {
    size_t i3 = i * 3;
    for (size_t j = 0; j < 3; j++) {
      if (frame->data_provider->sensor_name() == "front_12mm") {
        K_data[i3 + j] = camera_k_matrix(i, j) * 2.f;
      } else {
        K_data[i3 + j] = camera_k_matrix(i, j);
      }
    }
  }
  AINFO << "Camera k matrix input to obstacle postprocessor: \n"
        << K_data[0] << ", " << K_data[1] << ", " << K_data[2] << "\n"
        << K_data[3] << ", " << K_data[4] << ", " << K_data[5] << "\n"
        << K_data[6] << ", " << K_data[7] << ", " << K_data[8] << "\n";
  ratio_data[0] = 4.f * static_cast<float>(frame->data_provider->src_width()) /
                  static_cast<float>(width_);
  ratio_data[1] = 4.f * static_cast<float>(frame->data_provider->src_height()) /
                  static_cast<float>(height_);

  AINFO << "Start: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;
  image_options.crop_roi = base::RectI(
      0, offset_y_, static_cast<int>(base_camera_model_->get_width()),
      static_cast<int>(base_camera_model_->get_height()) - offset_y_);
  image_options.do_crop = true;
  frame->data_provider->GetImage(image_options, image_.get());
  AINFO << "GetImageBlob: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  // todo(zero): need to modify to cuda code
  Preprocessor(image_.get(), input_blob);

  AINFO << "Camera type: " << frame->data_provider->sensor_name();

  inference_->Infer();
  AINFO << "Network Forward: " << static_cast<double>(timer.Toc()) * 0.001
        << "ms";
  get_smoke_objects_cpu(smoke_blobs_, types_, smoke_param_.model_param(),
                        light_vis_conf_threshold_, light_swt_conf_threshold_,
                        overlapped_.get(), idx_sm_.get(),
                        &(frame->detected_objects),
                        frame->data_provider->src_width(),
                        frame->data_provider->src_height() - offset_y_);

  AINFO << "GetObj: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  filter_bbox(min_dims_, &(frame->detected_objects));
  FeatureExtractorOptions feature_options;
  feature_options.normalized = true;
  AINFO << "Post1: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  feature_extractor_->Extract(feature_options, frame);
  AINFO << "Extract: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  recover_smoke_bbox(frame->data_provider->src_width(),
                     frame->data_provider->src_height() - offset_y_, offset_y_,
                     &frame->detected_objects);

  // post processing
  int left_boundary =
      static_cast<int>(border_ratio_ * static_cast<float>(image_->cols()));
  int right_boundary = static_cast<int>((1.0f - border_ratio_) *
                                        static_cast<float>(image_->cols()));
  for (auto &obj : frame->detected_objects) {
    // recover alpha
    obj->camera_supplement.alpha /= ori_cycle_;
    // get area_id from visible_ratios
    if (smoke_param_.model_param().num_areas() == 0) {
      obj->camera_supplement.area_id =
          get_area_id(obj->camera_supplement.visible_ratios);
    }
    // clear cut off ratios
    auto &box = obj->camera_supplement.box;
    if (box.xmin >= left_boundary) {
      obj->camera_supplement.cut_off_ratios[2] = 0;
    }
    if (box.xmax <= right_boundary) {
      obj->camera_supplement.cut_off_ratios[3] = 0;
    }
  }
  AINFO << "Post2: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  return true;
}

bool SmokeObstacleDetector::Process(DataFrame *data_frame) {
  if (data_frame == nullptr) {
    return false;
  }
  auto frame = data_frame->camera_frame;

  Timer timer;
  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to " << gpu_id_;
    return false;
  }
  const auto &camera_k_matrix = frame->camera_k_matrix.inverse();
  auto const &net_param = smoke_param_.net_param();
  auto input_blob = inference_->get_blob(net_param.input_data_blob());
  auto input_K_blob = inference_->get_blob(net_param.input_instric_blob());
  auto input_ratio_blob = inference_->get_blob(net_param.input_ratio_blob());

  input_K_blob->Reshape({1, 3, 3});
  input_ratio_blob->Reshape({1, 2});

  float *ratio_data = input_ratio_blob->mutable_cpu_data();
  float *K_data = input_K_blob->mutable_cpu_data();
  for (size_t i = 0; i < 3; i++) {
    size_t i3 = i * 3;
    for (size_t j = 0; j < 3; j++) {
      if (frame->data_provider->sensor_name() == "front_12mm") {
        K_data[i3 + j] = camera_k_matrix(i, j) * 2.f;
      } else {
        K_data[i3 + j] = camera_k_matrix(i, j);
      }
    }
  }
  AINFO << "Camera k matrix input to obstacle postprocessor: \n"
        << K_data[0] << ", " << K_data[1] << ", " << K_data[2] << "\n"
        << K_data[3] << ", " << K_data[4] << ", " << K_data[5] << "\n"
        << K_data[6] << ", " << K_data[7] << ", " << K_data[8] << "\n";
  ratio_data[0] = 4.f * static_cast<float>(frame->data_provider->src_width()) /
                  static_cast<float>(width_);
  ratio_data[1] = 4.f * static_cast<float>(frame->data_provider->src_height()) /
                  static_cast<float>(height_);

  AINFO << "Start: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;
  image_options.crop_roi = base::RectI(
      0, offset_y_, static_cast<int>(base_camera_model_->get_width()),
      static_cast<int>(base_camera_model_->get_height()) - offset_y_);
  image_options.do_crop = true;
  frame->data_provider->GetImage(image_options, image_.get());
  AINFO << "GetImageBlob: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  // todo(zero): need to modify to cuda code
  Preprocessor(image_.get(), input_blob);

  AINFO << "Camera type: " << frame->data_provider->sensor_name();

  inference_->Infer();
  AINFO << "Network Forward: " << static_cast<double>(timer.Toc()) * 0.001
        << "ms";
  get_smoke_objects_cpu(smoke_blobs_, types_, smoke_param_.model_param(),
                        light_vis_conf_threshold_, light_swt_conf_threshold_,
                        overlapped_.get(), idx_sm_.get(),
                        &(frame->detected_objects),
                        frame->data_provider->src_width(),
                        frame->data_provider->src_height() - offset_y_);
  AINFO << "GetObj: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  filter_bbox(min_dims_, &(frame->detected_objects));
  FeatureExtractorOptions feature_options;
  feature_options.normalized = true;
  AINFO << "Post1: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  feature_extractor_->Extract(feature_options, frame);
  AINFO << "Extract: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";
  recover_smoke_bbox(frame->data_provider->src_width(),
                     frame->data_provider->src_height() - offset_y_, offset_y_,
                     &frame->detected_objects);

  // post processing
  int left_boundary =
      static_cast<int>(border_ratio_ * static_cast<float>(image_->cols()));
  int right_boundary = static_cast<int>((1.0f - border_ratio_) *
                                        static_cast<float>(image_->cols()));
  for (auto &obj : frame->detected_objects) {
    // recover alpha
    obj->camera_supplement.alpha /= ori_cycle_;
    // get area_id from visible_ratios
    if (smoke_param_.model_param().num_areas() == 0) {
      obj->camera_supplement.area_id =
          get_area_id(obj->camera_supplement.visible_ratios);
    }
    // clear cut off ratios
    auto &box = obj->camera_supplement.box;
    if (box.xmin >= left_boundary) {
      obj->camera_supplement.cut_off_ratios[2] = 0;
    }
    if (box.xmax <= right_boundary) {
      obj->camera_supplement.cut_off_ratios[3] = 0;
    }
  }
  AINFO << "Post2: " << static_cast<double>(timer.Toc()) * 0.001 << "ms";

  return true;
}

REGISTER_OBSTACLE_DETECTOR(SmokeObstacleDetector);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
