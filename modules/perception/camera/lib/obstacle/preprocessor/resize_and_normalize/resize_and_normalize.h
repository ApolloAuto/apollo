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
#pragma once

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace camera {

class ReSizeAndNormalize : public pipeline::Plugin {
 public:
  using PluginConfig = pipeline::PluginConfig;

 public:
  ReSizeAndNormalize() = default;

  explicit ReSizeAndNormalize(const PluginConfig& plugin_config);

  virtual ~ReSizeAndNormalize() = default;

  bool Process(const cv::Mat &im, float *image_data_array);

  bool Init(const PluginConfig& plugin_config) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  void Resize(const cv::Mat &img, int resized_h, int resized_w,
              cv::Mat *resize_img);

void Normalize(const std::vector<float> &mean,
                                   const std::vector<float> &std, float scale,
                                   cv::Mat *im);

  void Mat2Vec(const cv::Mat &im, float *image_data_array);

  int resized_width_;
  int resized_height_;
  std::vector<float> mean_;
  std::vector<float> std_;
  float scale_;
};  // class ReSizeAndNormalize

}  // namespace camera
}  // namespace perception
}  // namespace apollo
