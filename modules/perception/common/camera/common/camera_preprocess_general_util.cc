/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/common/camera/common/camera_preprocess_general_util.h"

#include <math.h>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

cv::Mat GetImage(const std::string &path, bool to_rgb) {
  cv::Mat img = cv::imread(path);

  if (!img.data) {
    throw std::runtime_error("Failed to read image: " + path);
  }

  if (to_rgb && img.channels() == 3) {
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  }
  return img;
}

// Resize use opencv lib
void Resize(const cv::Mat &input_img, cv::Mat *out_img, int width, int height,
            double fx, double fy, int interpolation) {
  ACHECK(nullptr != out_img);
  cv::resize(input_img, *out_img, cv::Size(width, height), fx, fy,
             interpolation);
}

void Normalize(const std::vector<float> &mean, const std::vector<float> &std,
               float scale, cv::Mat *img) {
  float eps = 1e-6;
  ACHECK(nullptr != img);
  ACHECK(3 == std.size());
  for (const auto std_value : std) {
    ACHECK(fabs(std_value) > eps);
  }
  ACHECK(fabs(scale) > eps);
  (*img).convertTo(*img, CV_32FC3, scale);
  for (int h = 0; h < img->rows; h++) {
    for (int w = 0; w < img->cols; w++) {
      img->at<cv::Vec3f>(h, w)[0] =
          (img->at<cv::Vec3f>(h, w)[0] - mean[0]) / std[0];
      img->at<cv::Vec3f>(h, w)[1] =
          (img->at<cv::Vec3f>(h, w)[1] - mean[1]) / std[1];
      img->at<cv::Vec3f>(h, w)[2] =
          (img->at<cv::Vec3f>(h, w)[2] - mean[2]) / std[2];
    }
  }
}

std::vector<float> HWC2CHW(const cv::Mat &input_img) {
  int channel = input_img.channels();
  int width = input_img.cols;
  int height = input_img.rows;

  std::vector<cv::Mat> input_channels(channel);
  cv::split(input_img, input_channels);

  std::vector<float> result(channel * width * height);
  auto data = result.data();
  int channel_length = width * height;

  for (int i = 0; i < channel; ++i) {
    memcpy(data, input_channels[i].data, channel_length * sizeof(float));
    data += channel_length;
  }
  return result;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
