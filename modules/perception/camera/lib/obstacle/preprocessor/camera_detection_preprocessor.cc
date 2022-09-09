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

namespace apollo {
namespace perception {
namespace camera {

bool CameraDetectionPreprocessor::Init(const StageConfig& stage_config) {
  ACHECK(stage_config.has_camera_detection_preprocessor());
  camera_detection_preprocessor_config_ =
      stage_config.camera_detection_preprocessor();

  get_image_data_ = PluginFactory::CreatePlugin(stage_config.get_image_data());
  resize_and_normalize_ =
      PluginFactory::CreatePlugin(stage_config.resize_and_normalize());
  if (!get_image_data_->Init(stage_config.get_image_data())) {
    return false;
  }
  if (!resize_and_normalize_->Init(stage_config.resize_and_normalize())) {
    return false;
  }
  return true;
}

bool CameraDetectionPreprocessor::Process(DataFrame* data_frame) {
  return true;
}
// input: data_frame
// output:
//          resize、normalize之后的 image_data_array -> input_blob(smoke detect
//          stage)
//          k_inv -> input_k_blob(smoke detect stage)

bool CameraDetectionPreprocessor::Process(DataFrame* data_frame, float * k_inv, float * image_data_array) {
  if (nullptr == data_frame) {
    AERROR << "Input null data_frame ptr.";
    return false;
  }
  if (nullptr == image) {
    AERROR << "Input null image ptr.";
    return false;
  }
  if (nullptr == k_inv) {
    AERROR << "Input null k_inv ptr.";
    return false;
  }

  auto camera_frame = data_frame->camera_frame;
  cv::Mat image_cv;
  if (!get_image_data_->Process(*camera_frame, k_inv, &image_cv)){
    return false;
  }

  if (!resize_and_normalize_->Process(image_cv,  image_data_array)){
    return false;
  }

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
