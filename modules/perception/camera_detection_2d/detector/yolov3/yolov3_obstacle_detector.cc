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
#include "modules/perception/camera_detection_2d/detector/yolov3/yolov3_obstacle_detector.h"

#include <opencv2/opencv.hpp>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera_detection_2d/detector/yolov3/postprocess.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

using cyber::common::GetAbsolutePath;

void Yolov3ObstacleDetector::LoadInputShape(
    const yolov3::ModelParam &model_param) {
  // loading config params
  width_ = model_param.resize().width();
  height_ = model_param.resize().height();

  image_width_ = options_.image_width;
  image_height_ = options_.image_height;

  AINFO << " image_height =" << image_height_
        << ", image_width=" << image_width_ << ", height=" << height_
        << ", width=" << width_;
}

void Yolov3ObstacleDetector::LoadParam(const yolov3::ModelParam &model_param) {
  confidence_threshold_ = model_param.confidence_threshold();
  border_ratio_ = model_param.border_ratio();

  // Init NMS proto param by config file
  const auto &nms_param = model_param.nms_param();
  nms_.set_sigma(nms_param.sigma());
  nms_.set_type(nms_param.type());
  nms_.set_threshold(nms_param.threshold());
}

bool Yolov3ObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  options_ = options;

  gpu_id_ = options.gpu_id;
  BASE_CUDA_CHECK(cudaSetDevice(gpu_id_));
  BASE_CUDA_CHECK(cudaStreamCreate(&stream_));

  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &model_param_)) {
    AERROR << "Read proto_config failed! " << config_file;
    return false;
  }

  LoadInputShape(model_param_);
  LoadParam(model_param_);

  const auto &model_info = model_param_.info();
  std::string model_path = GetModelPath(model_info.name());
  if (!InitNetwork(model_info, model_path)) {
    AERROR << "Init network failed!";
    return false;
  }
  AERROR << "[INFO] yolov3 2D model init success";
  return true;
}

bool Yolov3ObstacleDetector::Preprocess(const base::Image8U *image,
                                        base::BlobPtr<float> input_blob) {
  ACHECK(image != nullptr);
  ACHECK(input_blob != nullptr);

  // init cv img containter, same to image row and col
  cv::Mat img = cv::Mat(image->rows(), image->cols(), CV_8UC3);
  memcpy(img.data, image->cpu_data(),
         image->rows() * image->cols() * image->channels() * sizeof(uint8_t));
  // tools : image show to help check
  // cv::imwrite("yolov3_image.png", img);

  // generate new pure black bg as same size as ratio
  float ratio = std::max(image_width_, image_height_);
  int dim_diff = std::abs(image_height_ - image_width_);
  int pad1 = std::floor(static_cast<double>(dim_diff) / 2.0);
  cv::Mat out(ratio, ratio, CV_8UC3, {0, 0, 0});  // helps to clear noisy

  // image preprocess sames to training
  // if h < w : add img to middle of bg
  // else : add img to left of bg
  // add img to bg, avoid image distortion
  img.copyTo(out(cv::Rect(0, pad1, img.cols, img.rows)));

  // cv::resize(out, out, cv::Size(width_, height_), cv::INTER_NEAREST);
  cv::resize(out, out, cv::Size(width_, height_), cv::INTER_AREA);

  img = out;

  // normallize channel value from 0～255 to 0~1 and change it to float type
  img.convertTo(img, CV_32F, 1.0 / 255, 0);

  int model_input_rows = img.rows;
  int model_input_cols = img.cols;
  int model_input_chs = img.channels();

  // fill input_blob -> tensor_image will be used by model
  input_blob->Reshape({1, model_input_chs, model_input_rows, model_input_cols});
  float *input_data = input_blob->mutable_cpu_data();
  for (int i = 0; i < model_input_chs; ++i) {
    // put img model_input_chs i data to input_blob
    cv::extractChannel(
        img,
        cv::Mat(model_input_rows, model_input_cols, CV_32FC1,
                input_data + i * model_input_rows * model_input_cols),
        i);
  }
  return true;
}

bool Yolov3ObstacleDetector::Detect(onboard::CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }

  if (cudaSetDevice(gpu_id_) != cudaSuccess) {
    AERROR << "Failed to set device to " << gpu_id_;
    return false;
  }

  auto model_inputs = model_param_.info().inputs();
  auto input_blob = net_->get_blob(model_inputs[0].name());

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;

  base::Image8U image;
  frame->data_provider->GetImage(image_options, &image);

  // TODO(lordon): use cuda
  Preprocess(&image, input_blob);

  // model infer, save to output blob
  net_->Infer();

  // get objects from network inference result
  auto model_outputs = model_param_.info().outputs();
  auto blob_predict = net_->get_blob(model_outputs[0].name());

  GetYolov3ObjectsCpu(blob_predict, model_param_, nms_, width_, height_,
                      image_width_, image_height_, &frame->detected_objects);

  frame->feature_blob = net_->get_blob(model_outputs[1].name());

  // post processing
  float border_ratio = 0.01;
  int left_boundary =
      static_cast<int>(border_ratio * static_cast<float>(image.cols()));
  int right_boundary = static_cast<int>((1.0f - border_ratio) *
                                        static_cast<float>(image.cols()));
  for (auto &obj : frame->detected_objects) {
    obj->camera_supplement.area_id = 1;

    // clear cut off ratios
    auto &box = obj->camera_supplement.box;
    if (box.xmin >= left_boundary) {
      obj->camera_supplement.cut_off_ratios[2] = 0;
    }
    if (box.xmax <= right_boundary) {
      obj->camera_supplement.cut_off_ratios[3] = 0;
    }
  }

  return true;
}
REGISTER_OBSTACLE_DETECTOR(Yolov3ObstacleDetector);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
