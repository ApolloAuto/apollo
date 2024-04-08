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

#include "modules/perception/lidar_detection/lidar_detection_component.h"

#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"

namespace apollo {
namespace perception {
namespace lidar {

bool LidarDetectionComponent::Init() {
  LidarDetectionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get LidarDetectionComponentConfig file failed";
    return false;
  }
  AINFO << "Lidar Detection Component Configs: " << comp_config.DebugString();

  sensor_name_ = comp_config.sensor_name();
  // writer
  output_channel_name_ = comp_config.output_channel_name();
  writer_ =
      node_->CreateWriter<onboard::LidarFrameMessage>(output_channel_name_);

  use_object_builder_ = comp_config.use_object_builder();

  // detector init
  auto plugin_param = comp_config.plugin_param();
  std::string detector_name = plugin_param.name();
  BaseLidarDetector* detector =
      BaseLidarDetectorRegisterer::GetInstanceByName(detector_name);
  CHECK_NOTNULL(detector);
  detector_.reset(detector);

  LidarDetectorInitOptions detection_init_options;
  detection_init_options.sensor_name = sensor_name_;
  detection_init_options.config_path = plugin_param.config_path();
  detection_init_options.config_file = plugin_param.config_file();
  ACHECK(detector_->Init(detection_init_options))
      << "lidar detector init error";

  // TODO(zero): Fix paddle doesn't output log to file
  FLAGS_logtostderr = 0;

  // object builder init
  if (use_object_builder_) {
    ObjectBuilderInitOptions builder_init_options;
    ACHECK(builder_.Init(builder_init_options));
  }

  AINFO << "Successfully init lidar detection component.";
  return true;
}

bool LidarDetectionComponent::Proc(
    const std::shared_ptr<LidarFrameMessage>& message) {
  PERF_FUNCTION()
  // internal proc
  bool status = InternalProc(message);
  if (status) {
    writer_->Write(message);
    AINFO << "Send Lidar detection output message.";
  }
  return status;
}

bool LidarDetectionComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& in_message) {
  // detector
  PERF_BLOCK("lidar_detector")
  LidarDetectorOptions detection_options;
  if (!detector_->Detect(detection_options, in_message->lidar_frame_.get())) {
    AERROR << "Lidar detector detect error!";
    return false;
  }
  PERF_BLOCK_END

  // object builder
  PERF_BLOCK("object_builder")
  if (use_object_builder_) {
    ObjectBuilderOptions builder_options;
    if (!builder_.Build(builder_options, in_message->lidar_frame_.get())) {
      AERROR << "Lidar detector, object builder error.";
      return false;
    }
  }
  PERF_BLOCK_END

  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
