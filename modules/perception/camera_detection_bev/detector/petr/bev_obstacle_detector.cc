/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_detection_bev/detector/petr/bev_obstacle_detector.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera_detection_bev/detector/petr/postprocess.h"
#include "modules/perception/camera_detection_bev/detector/petr/preprocess.h"
#include "modules/perception/common/camera/common/util.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {


void BEVObstacleDetector::InitImageSize(const petr::ModelParam &model_param) {
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

bool BEVObstacleDetector::Init(const ObstacleDetectorInitOptions &options) {
  options_ = options;

  std::string config_file =
      GetConfigFile(options_.config_path, options_.config_file);
  if (!cyber::common::GetProtoFromFile(config_file, &model_param_)) {
    AERROR << "Read model param failed!";
    return false;
  }

  InitImageSize(model_param_);
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

void BEVObstacleDetector::Mat2Vec(const cv::Mat &im, float *data) {
  ACHECK(nullptr != data);
  int rh = im.rows;
  int rw = im.cols;
  int rc = im.channels();

  for (int i = 0; i < rc; ++i) {
    cv::extractChannel(im, cv::Mat(rh, rw, CV_32FC1, data + i * rh * rw), i);
  }
}

bool BEVObstacleDetector::ImagePreprocess(const CameraFrame *frame,
                                          base::BlobPtr<float> input_img_blob) {
  float *img_data_ptr = input_img_blob->mutable_cpu_data();

  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::RGB;

  float scale = 1.0f;
  int camera_num = 6;
  base::Image8U image;
  for (int i = 0; i < camera_num; ++i) {
    frame->data_provider[i]->GetImage(image_options, &image);
    cv::Mat img(image.rows(), image.cols(), CV_8UC3, cv::Scalar(0, 0, 0));
    memcpy(img.data, image.cpu_data(), image.total() * sizeof(uint8_t));

    Resize(&img, width_, height_);

    common::Crop crop = model_param_.crop();
    Crop(&img, crop.x(), crop.y(), crop.width(), crop.height());

    common::Normalize normalize = model_param_.normalize();
    std::vector<float> mean{normalize.mean().begin(), normalize.mean().end()};
    std::vector<float> std{normalize.std().begin(), normalize.std().end()};
    Normalize(mean, std, scale, &img);

    std::vector<float> image_data;
    Mat2Vec(img, image_data.data());

    auto length = image_data.size();
    memcpy(img_data_ptr + i * length, image_data.data(),
           length * sizeof(float));
  }

  return true;
}

bool BEVObstacleDetector::ImageExtrinsicPreprocess(
    base::BlobPtr<float> input_img2lidar_blob) {
  float *img2lidar_data_ptr = input_img2lidar_blob->mutable_cpu_data();
  memcpy(img2lidar_data_ptr, k_data_.data(), k_data_.size() * sizeof(float));
  return true;
}

bool BEVObstacleDetector::Detect(CameraFrame *frame) {
  if (frame == nullptr) {
    return false;
  }

  // Inputs
  auto model_inputs = model_param_.info().inputs();
  auto input_img_blob = net_->get_blob(model_inputs[0].name());
  auto input_img2lidar_blob = net_->get_blob(model_inputs[1].name());

  ImagePreprocess(frame, input_img_blob);
  ImageExtrinsicPreprocess(input_img2lidar_blob);

  // Net forward
  net_->Infer();

  // Outputs
  auto model_outputs = model_param_.info().outputs();
  auto output_bbox_blob = net_->get_blob(model_outputs[0].name());
  auto output_score_blob = net_->get_blob(model_outputs[1].name());
  auto output_label_blob = net_->get_blob(model_outputs[2].name());

  float threshold = model_param_.score_threshold();
  GetObjects(output_bbox_blob, output_label_blob, output_score_blob, types_,
             threshold, &frame->detected_objects);

  Nuscenes2Apollo(&frame->detected_objects);

  return true;
}

/*
bbox_nuscenes to bbox_apollo: Rotate 90 degrees counterclockwise about the
z-axis
*/
// bbox: x, y, z, w, l, h, yaw, vx, vy
bool BEVObstacleDetector::Nuscenes2Apollo(
    std::vector<base::ObjectPtr> *objects) {
  ACHECK(objects != nullptr);
  for (auto &obj : *objects) {
    float theta = obj->theta;
    theta -= M_PI / 2;
    theta = std::atan2(sinf(theta), cosf(theta));
    theta = -theta;

    obj->theta = theta;
    obj->direction[0] = cosf(theta);
    obj->direction[1] = sinf(theta);
    obj->direction[2] = 0;

    Eigen::AngleAxisd rotation_vector(M_PI / 2, Eigen::Vector3d(1, 0, 0));
    obj->center = rotation_vector.matrix() * obj->center;
    obj->camera_supplement.local_center[0] = static_cast<float>(obj->center[0]);
    obj->camera_supplement.local_center[1] = static_cast<float>(obj->center[1]);
    obj->camera_supplement.local_center[2] = static_cast<float>(obj->center[2]);
  }
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
