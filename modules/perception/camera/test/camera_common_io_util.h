/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include "gtest/gtest.h"

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

inline void save_image(const std::string &path, base::Image8U *image) {
  AINFO << path;
  int cv_type = image->type() == base::Color::GRAY ? CV_8UC1 : CV_8UC3;
  cv::Mat cv_img(image->rows(), image->cols(), cv_type,
                 image->mutable_cpu_data(), image->width_step());
  cv::imwrite(path, cv_img);
}

inline void save_blob(const std::string &path, base::Blob<uint8_t> *blob) {
  AINFO << path;
  blob->Reshape({blob->shape(1), blob->shape(2), blob->shape(3)});
  int cv_type = blob->shape(2) == 1 ? CV_8UC1 : CV_8UC3;
  cv::Mat cv_img(blob->shape(0), blob->shape(1), cv_type,
                 blob->mutable_cpu_data());
  cv::imwrite(path, cv_img);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
