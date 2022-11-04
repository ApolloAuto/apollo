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

#include <string>
#include <memory>

#include <opencv2/opencv.hpp>

#include "modules/perception/pipeline/data_frame.h"
#include "modules/perception/pipeline/plugin.h"

namespace apollo {
namespace perception {
namespace camera {

class GetImageData : public pipeline::Plugin {
 public:
  using PluginConfig = pipeline::PluginConfig;
  using DataFrame = pipeline::DataFrame;

 public:
  GetImageData() = default;

  explicit GetImageData(const PluginConfig& plugin_config);

  virtual ~GetImageData() = default;

  bool Init(const PluginConfig& plugin_config) override;

  bool Process(DataFrame* data_frame, float* k_inv, cv::Mat* imag_cv);

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  bool GetKInverse(const CameraFrame& frame, float* k_inv);

  bool GetImage(const CameraFrame& frame, cv::Mat* image_cv);

 private:
  int image_origin_width_;
  int image_origin_height_;
  int image_origin_channel_;

  float offset_y_ = 0.0;

  std::shared_ptr<base::BaseCameraModel> base_camera_model_ = nullptr;

  std::shared_ptr<base::Image8U> image_ = nullptr;
};  // class GetImageData

}  // namespace camera
}  // namespace perception
}  // namespace apollo
