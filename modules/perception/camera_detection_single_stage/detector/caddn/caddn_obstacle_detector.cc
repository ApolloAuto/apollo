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
#include "modules/perception/camera_detection_single_stage/detector/caddn/caddn_obstacle_detector.h"

#include <memory>

#include "opencv2/opencv.hpp"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera_detection_single_stage/detector/caddn/postprocess.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {


void CaddnObstacleDetector::InitImageSize(
    const caddn::ModelParam &model_param) {
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

void CaddnObstacleDetector::InitParam(const caddn::ModelParam &model_param) {
  // confidence threshold
  score_threshold_ = model_param.score_threshold();
}

bool CaddnObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  options_ = options;
  std::string config_file =
      GetConfigFile(options_.config_path, options_.config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &model_param_)) {
    AERROR << "Read model param failed!";
    return false;
  }

  InitImageSize(model_param_);
  InitParam(model_param_);

  const auto &model_info = model_param_.info();
  std::string model_path = GetModelPath(model_info.name());
  std::string types_file =
      GetModelFile(model_info.name(), model_info.types_file().file());
  if (!LoadTypes(types_file, &types_)) {
    AERROR << "Load types file failed!";
    return false;
  }

  if (!InitNetwork(model_param_.info(), model_path)) {
    AERROR << "Init network failed!";
    return false;
  }
  return true;
}

bool CaddnObstacleDetector::Preprocess(const base::Image8U *image,
                                       base::BlobPtr<float> input_blob) {
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

bool CaddnObstacleDetector::Detect(onboard::CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }

  // Inputs
  auto model_inputs = model_param_.info().inputs();
  auto input_image_blob = net_->get_blob(model_inputs[0].name());
  auto input_cam2img_blob = net_->get_blob(model_inputs[1].name());
  auto input_lidar2cam_blob = net_->get_blob(model_inputs[2].name());

  const auto &camera_k_matrix = options_.intrinsic;
  float *input_cam_intrinsic = input_cam2img_blob->mutable_cpu_data();
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      if (3 == j) {
        input_cam_intrinsic[i * 4 + j] = 0.0;
      } else {
        input_cam_intrinsic[i * 4 + j] = camera_k_matrix(i, j);
      }
    }
  }

  float *input_lidar2cam_data = input_lidar2cam_blob->mutable_cpu_data();
  for (int i = 0; i < 16; ++i) {
    input_lidar2cam_data[i] = input_lidar_data_[i];
  }

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  std::shared_ptr<base::Image8U> image = std::make_shared<base::Image8U>();
  frame->data_provider->GetImage(image_options, image.get());

  Preprocess(image.get(), input_image_blob);

  // Infer
  net_->Infer();

  // Outputs
  auto model_outputs = model_param_.info().outputs();
  auto out_detections = net_->get_blob(model_outputs[0].name());
  auto out_labels = net_->get_blob(model_outputs[1].name());
  auto out_scores = net_->get_blob(model_outputs[2].name());

  // todo(daohu527): The caddn model currently does not output tracking features
  // appearance features for tracking
  // frame->feature_blob = net_->get_blob(model_outputs[3].name());

  GetCaddnObjects(&frame->detected_objects, model_param_, types_,
                  out_detections, out_labels, out_scores);

  RecoverCaddnBbox(frame->data_provider->src_width(),
                   frame->data_provider->src_height(), 0,
                   &frame->detected_objects);

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
