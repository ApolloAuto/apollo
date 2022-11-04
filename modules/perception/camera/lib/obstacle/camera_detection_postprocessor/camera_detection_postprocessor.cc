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

#include "modules/perception/camera/lib/obstacle/camera_detection_postprocessor/camera_detection_postprocessor.h"

namespace apollo {
namespace perception {
namespace camera {

bool CameraDetectionPostprocessor::Init(const StageConfig &stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  ACHECK(stage_config.has_camera_detection_postprocessor_config());
  camera_detection_postprocessor_config_ =
      stage_config.camera_detection_postprocessor_config();

  // camera_get_object_ =
  //     pipeline::dynamic_unique_cast<GetObject>(
  //         pipeline::PluginFactory::CreatePlugin(
  //             plugin_config_map_[PluginType::POINTCLOUD_GET_OBJECTS])
  //     );

  // if (!camera_get_object_->Init(stage_config.get_object())) {
  //   return false;
  // }

  return true;
}

bool CameraDetectionPostprocessor::Process(DataFrame *data_frame) {
  return true;
}

bool CameraDetectionPostprocessor::Process(
    const std::vector<float> &detect_result,
    const std::vector<base::ObjectSubType> &types, DataFrame *data_frame) {
  if (nullptr == data_frame) {
    AERROR << "Input null data_frame ptr.";
    return false;
  }
  // todo(zero): need fix
  // if (nullptr == image) {
  //   AERROR << "Input null image ptr.";
  //   return false;
  // }
  // if (nullptr == k_inv) {
  //   AERROR << "Input null k_inv ptr.";
  //   return false;
  // }

  // if (!camera_get_object_->Process(detect_result, types, data_frame)){
  //   return false;
  // }

  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
