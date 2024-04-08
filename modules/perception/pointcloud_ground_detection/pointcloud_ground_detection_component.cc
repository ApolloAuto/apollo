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

#include "modules/perception/pointcloud_ground_detection/pointcloud_ground_detection_component.h"

#include "cyber/profiler/profiler.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;

bool PointCloudGroundDetectComponent::Init() {
  PointCloudGroundDetectComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get PointCloudGroundDetectComponentConfig file failed";
    return false;
  }
  AINFO << "PointCloud Ground Detect Component Configs: "
        << comp_config.DebugString();
  output_channel_name_ = comp_config.output_channel_name();
  writer_ =
      node_->CreateWriter<onboard::LidarFrameMessage>(output_channel_name_);

  // groun detector
  auto plugin_param = comp_config.plugin_param();
  std::string ground_detector_name = plugin_param.name();
  ground_detector_ =
      BaseGroundDetectorRegisterer::GetInstanceByName(ground_detector_name);
  CHECK_NOTNULL(ground_detector_);

  GroundDetectorInitOptions ground_detector_init_options;
  ground_detector_init_options.config_path = plugin_param.config_path();
  ground_detector_init_options.config_file = plugin_param.config_file();
  ACHECK(ground_detector_->Init(ground_detector_init_options))
      << "Failed to init ground detection.";
  return true;
}

bool PointCloudGroundDetectComponent::Proc(
    const std::shared_ptr<LidarFrameMessage>& message) {
  PERF_FUNCTION()
  // internal proc
  bool status = InternalProc(message);
  if (status) {
    writer_->Write(message);
    AINFO << "Send pointcloud ground detect output message.";
  }
  return true;
}

bool PointCloudGroundDetectComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& message) {
  auto lidar_frame_ref = message->lidar_frame_;
  PERF_BLOCK("ground_detector")
  GroundDetectorOptions ground_detector_options;
  if (!ground_detector_->Detect(
      ground_detector_options, lidar_frame_ref.get())) {
    AERROR << "Ground detect error.";
    return false;
  }
  PERF_BLOCK_END

  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
