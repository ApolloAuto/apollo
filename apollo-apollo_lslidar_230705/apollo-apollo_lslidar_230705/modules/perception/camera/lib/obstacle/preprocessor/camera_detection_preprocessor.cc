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

#include "modules/perception/camera/lib/obstacle/preprocessor/camera_detection_preprocessor.h"

#include "modules/perception/pipeline/plugin_factory.h"

namespace apollo {
namespace perception {
namespace camera {

bool CameraDetectionPreprocessor::Init(const StageConfig& stage_config) {
  ACHECK(stage_config.has_camera_detection_preprocessor_config());
  if (!Initialize(stage_config)) {
    return false;
  }

  get_image_data_ =
      pipeline::dynamic_unique_cast<GetImageData>(
          pipeline::PluginFactory::CreatePlugin(
              plugin_config_map_[PluginType::GET_IMAGE_DATA]));
  CHECK_NOTNULL(get_image_data_);

  resize_and_normalize_ =
      pipeline::dynamic_unique_cast<ReSizeAndNormalize>(
          pipeline::PluginFactory::CreatePlugin(
              plugin_config_map_[PluginType::RESIZIE_AND_NORMALIZE]));
  CHECK_NOTNULL(resize_and_normalize_);

  return true;
}

bool CameraDetectionPreprocessor::Process(DataFrame* data_frame) {
  return true;
}

bool CameraDetectionPreprocessor::Process(DataFrame* data_frame, float* k_inv,
                                          float* image_data_array) {
  if (nullptr == data_frame) {
    AERROR << "Input null data_frame ptr.";
    return false;
  }
  if (nullptr == k_inv) {
    AERROR << "Input null k_inv ptr.";
    return false;
  }
  if (nullptr == image_data_array) {
    AERROR << "Input null image_data_array ptr.";
    return false;
  }

  cv::Mat image_cv;
  if (!get_image_data_->Process(data_frame, k_inv, &image_cv)) {
    return false;
  }

  if (!resize_and_normalize_->Process(image_cv, image_data_array)) {
    return false;
  }

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
