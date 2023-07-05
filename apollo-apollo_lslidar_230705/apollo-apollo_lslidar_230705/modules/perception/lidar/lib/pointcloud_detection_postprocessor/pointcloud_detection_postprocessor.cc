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

#include "modules/perception/lidar/lib/pointcloud_detection_postprocessor/pointcloud_detection_postprocessor.h"

#include "modules/perception/pipeline/plugin_factory.h"

namespace apollo {
namespace perception {
namespace lidar {

// stage init
bool PointcloudDetectionPostprocessor::Init(const StageConfig& stage_config) {
  // ACHECK(stage_config.has_pointcloud_detection_postprocessor_config());

  if (!Initialize(stage_config)) {
    return false;
  }

  pointcloud_get_objects_ =
      pipeline::dynamic_unique_cast<PointCloudGetObjects>(
          pipeline::PluginFactory::CreatePlugin(
              plugin_config_map_[PluginType::POINTCLOUD_GET_OBJECTS]));
  CHECK_NOTNULL(pointcloud_get_objects_);

  return true;
}

bool PointcloudDetectionPostprocessor::Process(DataFrame* data_frame) {
  return true;
}

bool PointcloudDetectionPostprocessor::Process(
    const std::vector<float>& detections,
    const std::vector<int>& labels,
    DataFrame* data_frame) {
  if (nullptr == data_frame) {
    AERROR << "Input null data_frame ptr.";
    return false;
  }
  if (!pointcloud_get_objects_->Process(detections, labels, data_frame)) {
    return false;
  }
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
