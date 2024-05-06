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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace apollo {
namespace perception {
namespace camera {

cv::Mat GetImage(const std::string &path, bool to_rgb = false);

void Resize(const cv::Mat &input_img, cv::Mat *out_img, cv::Size size,
            double fx = 0, double fy = 0, int interpolation = cv::INTER_LINEAR);

void Normalize(const std::vector<float> &mean, const std::vector<float> &std,
               float scale, cv::Mat *img);

std::vector<float> HWC2CHW(const cv::Mat &input_img);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
