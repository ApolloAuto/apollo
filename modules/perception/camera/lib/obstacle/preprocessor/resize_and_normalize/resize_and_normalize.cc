/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/camera/lib/obstacle/preprocessor/resize_and_normalize/resize_and_normalize.h"


namespace apollo {
namespace perception {
namespace camera {

ReSizeAndNormalize::ReSizeAndNormalize(const PluginConfig& plugin_config) {
  Init(plugin_config);
}

bool ReSizeAndNormalize::Init(const PluginConfig &plugin_config) {
  ACHECK(plugin_config.has_resize_and_normalize_config());

  const auto& config = plugin_config.resize_and_normalize_config();
  resized_width_ = config.resized_width();
  resized_height_ = config.resized_height();
  mean_[0] = config.mean_r();
  mean_[1] = config.mean_g();
  mean_[2] = config.mean_b();
  std_[0] = config.std_r();
  std_[1] = config.std_g();
  std_[2] = config.std_b();
  scale_ = config.scale();
  return true;
}

bool ReSizeAndNormalize::Process(const cv::Mat &im, float *image_data_array) {
  cv::Mat resized_image;
  Resize(im, resized_height_, resized_width_, &resized_image);
  Normalize(mean_, std_, scale_, &resized_image);
  Mat2Vec(resized_image, image_data_array);
  return true;
}

void ReSizeAndNormalize::Resize(const cv::Mat &img, int resized_h,
                                int resized_w, cv::Mat *resize_img) {
  cv::resize(img, *resize_img, cv::Size(resized_h, resized_w), 0, 0,
             cv::INTER_LINEAR);
}

void ReSizeAndNormalize::Normalize(const std::vector<float> &mean,
                                   const std::vector<float> &std, float scale,
                                   cv::Mat *im) {
  if (scale) {
    (*im).convertTo(*im, CV_32FC3, scale);
  }
  for (int h = 0; h < im->rows; h++) {
    for (int w = 0; w < im->cols; w++) {
      im->at<cv::Vec3f>(h, w)[0] =
          (im->at<cv::Vec3f>(h, w)[0] - mean[0]) / std[0];
      im->at<cv::Vec3f>(h, w)[1] =
          (im->at<cv::Vec3f>(h, w)[1] - mean[1]) / std[1];
      im->at<cv::Vec3f>(h, w)[2] =
          (im->at<cv::Vec3f>(h, w)[2] - mean[2]) / std[2];
    }
  }
}

void ReSizeAndNormalize::Mat2Vec(const cv::Mat &im, float *image_data_array) {
  int rh = im.rows;
  int rw = im.cols;
  int rc = im.channels();
  for (int i = 0; i < rc; ++i) {
    cv::extractChannel(
        im, cv::Mat(rh, rw, CV_32FC1, image_data_array + i * rh * rw), i);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
