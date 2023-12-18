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
#include "modules/perception/camera_detection_single_stage/detector/smoke/smoke_obstacle_detector.h"

#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"
#include "modules/perception/camera_detection_single_stage/detector/smoke/postprocess.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

void SmokeObstacleDetector::InitImageOffset(
    const smoke::ModelParam &model_param) {
  // offset
  auto offset_ratio = model_param.offset_ratio();
  int image_width = options_.image_width;
  int image_height = options_.image_height;
  offset_y_ = static_cast<int>(image_height * offset_ratio.y());
  AINFO << "image_width=" << image_width << ", "
        << "image_height=" << image_height << ", "
        << "offset_y=" << offset_y_ << ", ";
}

void SmokeObstacleDetector::InitObstacleTypes() {
  types_.push_back(base::ObjectSubType::CAR);
  types_.push_back(base::ObjectSubType::CYCLIST);
  types_.push_back(base::ObjectSubType::PEDESTRIAN);
  types_.push_back(base::ObjectSubType::VAN);
  types_.push_back(base::ObjectSubType::TRUCK);
  types_.push_back(base::ObjectSubType::BUS);
  types_.push_back(base::ObjectSubType::TRAFFICCONE);
}

void SmokeObstacleDetector::InitImageSize(
    const smoke::ModelParam &model_param) {
  // resize
  auto resize = model_param.resize();
  if (resize.width() == 0 && resize.height() == 0) {
    width_ = options_.image_width * resize.fx();
    height_ = options_.image_height * resize.fy();
  } else {
    width_ = resize.width();
    height_ = resize.height();
  }

  AINFO << "height=" << height_ << ", "
        << "width=" << width_;
}

void SmokeObstacleDetector::InitParam(const smoke::ModelParam &model_param) {
  // min 2d height
  min_dims_ = model_param.min_dims();
  // confidence threshold
  confidence_threshold_ = model_param.confidence_threshold();
  // ori cycle
  ori_cycle_ = model_param.ori_cycle();
  // border ratio
  border_ratio_ = model_param.border_ratio();
}

bool SmokeObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  options_ = options;

  std::string config_file =
      GetConfigFile(options_.config_path, options_.config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &model_param_)) {
    AERROR << "Read model param failed!";
    return false;
  }

  InitImageOffset(model_param_);
  InitImageSize(model_param_);
  InitParam(model_param_);
  InitObstacleTypes();

  const auto &model_info = model_param_.info();
  std::string model_path = GetModelPath(model_info.name());

  if (!InitNetwork(model_param_.info(), model_path)) {
    AERROR << "Init network failed!";
    return false;
  }

  return true;
}

bool SmokeObstacleDetector::Preprocess(
    const base::Image8U *image, std::shared_ptr<base::Blob<float>> input_blob) {
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
  float *input_data = input_blob->mutable_cpu_data();
  for (int i = 0; i < chs; ++i) {
    cv::extractChannel(
        img, cv::Mat(rows, cols, CV_32FC1, input_data + i * rows * cols), i);
  }
  return true;
}

bool SmokeObstacleDetector::Detect(onboard::CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }

  if (cudaSetDevice(options_.gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to " << options_.gpu_id;
    return false;
  }

  // Check init height == runtime height
  if (options_.image_height != frame->data_provider->src_height() ||
      options_.image_width != frame->data_provider->src_width()) {
    AWARN << "Initialized image size not equal to runtime!";
  }

  auto model_inputs = model_param_.info().inputs();
  auto input_blob = net_->get_blob(model_inputs[0].name());
  auto input_k_blob = net_->get_blob(model_inputs[1].name());
  auto input_ratio_blob = net_->get_blob(model_inputs[2].name());

  // Input k matrix
  const auto &camera_k_matrix = options_.intrinsic.inverse();
  float *k_data = input_k_blob->mutable_cpu_data();
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      k_data[i * 3 + j] = camera_k_matrix(i, j);
    }
  }
  AINFO << "Camera k matrix input to obstacle postprocessor: \n"
        << k_data[0] << ", " << k_data[1] << ", " << k_data[2] << "\n"
        << k_data[3] << ", " << k_data[4] << ", " << k_data[5] << "\n"
        << k_data[6] << ", " << k_data[7] << ", " << k_data[8] << "\n";

  // Input ratio
  float *ratio_data = input_ratio_blob->mutable_cpu_data();
  ratio_data[0] = 4.f * static_cast<float>(frame->data_provider->src_width()) /
                  static_cast<float>(width_);
  ratio_data[1] = 4.f * static_cast<float>(frame->data_provider->src_height()) /
                  static_cast<float>(height_);

  // Input image
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;
  image_options.crop_roi = base::RectI(0, offset_y_, options_.image_width,
                                       options_.image_height - offset_y_);
  image_options.do_crop = true;
  // todo(daohu527) copy assignment
  image_ = std::make_shared<base::Image8U>();
  frame->data_provider->GetImage(image_options, image_.get());

  // Preprocess
  PERF_BLOCK("camera_3d_detector_preprocess")
  Preprocess(image_.get(), input_blob);
  PERF_BLOCK_END

  // Net forward
  PERF_BLOCK("camera_3d_detector_infer")
  net_->Infer();
  PERF_BLOCK_END

  // Output
  auto model_outputs = model_param_.info().outputs();
  auto detection_blob = net_->get_blob(model_outputs[0].name());

  PERF_BLOCK("camera_3d_detector_get_obj")
  GetObjectsCpu(detection_blob, types_, model_param_, &frame->detected_objects,
                frame->data_provider->src_width(),
                frame->data_provider->src_height() - offset_y_);
  PERF_BLOCK_END

  FilterByMinDims(min_dims_, &frame->detected_objects);

  RecoverBBox(frame->data_provider->src_width(),
              frame->data_provider->src_height() - offset_y_, offset_y_,
              &frame->detected_objects);

  // appearance features for tracking
  frame->feature_blob = net_->get_blob(model_outputs[1].name());

  // post processing
  for (auto &obj : frame->detected_objects) {
    // recover alpha
    obj->camera_supplement.alpha /= ori_cycle_;
  }

  return true;
}

REGISTER_OBSTACLE_DETECTOR(SmokeObstacleDetector);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
