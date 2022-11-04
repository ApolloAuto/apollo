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

#include "modules/perception/camera/lib/obstacle/preprocessor/get_image_data/get_image_data.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

GetImageData::GetImageData(const PluginConfig& plugin_config) {
  Init(plugin_config);
}

bool GetImageData::Init(const PluginConfig& plugin_config) {
  ACHECK(plugin_config.has_get_image_data_config());

  auto image_data_config = plugin_config.get_image_data_config();
  image_origin_width_ = image_data_config.image_origin_width();
  image_origin_height_ = image_data_config.image_origin_height();
  image_origin_channel_ = image_data_config.image_origin_channel();
  image_.reset(new base::Image8U(image_origin_height_, image_origin_width_,
                                 base::Color::RGB));
  return true;
}

bool GetImageData::Process(DataFrame* data_frame, float* k_inv,
                           cv::Mat* image_cv) {
  if (nullptr == data_frame) {
    AERROR << "Input null data_frame ptr.";
    return false;
  }
  if (nullptr == k_inv) {
    AERROR << "Input null k_inv ptr.";
    return false;
  }
  if (nullptr == image_cv) {
    AERROR << "Input null imag_cv ptr.";
    return false;
  }
  GetImage(*(data_frame->camera_frame), image_cv);
  GetKInverse(*(data_frame->camera_frame), k_inv);

  return true;
}

bool GetImageData::GetKInverse(const CameraFrame& frame, float* k_inv) {
  const auto& camera_k_matrix = frame.camera_k_matrix.inverse();
  for (size_t i = 0; i < 3; i++) {
    size_t i3 = i * 3;
    for (size_t j = 0; j < 3; j++) {
      if (frame.data_provider->sensor_name() == "front_12mm") {
        k_inv[i3 + j] = camera_k_matrix(i, j) * 2.f;
      } else {
        k_inv[i3 + j] = camera_k_matrix(i, j);
      }
    }
  }
  return true;
}

bool GetImageData::GetImage(const CameraFrame& frame, cv::Mat* image_cv) {
  DataProvider::ImageOptions image_options;
  image_options.target_color = base::Color::BGR;
  image_options.crop_roi = base::RectI(
      0, offset_y_, static_cast<int>(base_camera_model_->get_width()),
      static_cast<int>(base_camera_model_->get_height()) - offset_y_);
  image_options.do_crop = true;
  frame.data_provider->GetImage(image_options, image_.get());

  memcpy(image_cv->data, image_->cpu_data(),
         image_origin_width_ * image_origin_height_ * image_origin_channel_ *
             sizeof(uint8_t));

  return true;
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
