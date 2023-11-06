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

#include "modules/perception/lidar_detection_filter/lidar_detection_filter_component.h"

#include "cyber/common/log.h"
#include "cyber/profiler/profiler.h"

namespace apollo {
namespace perception {
namespace lidar {

bool LidarDetectionFilterComponent::Init() {
  LidarDetectionFilterComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get LidarDetectionFilterComponentConfig file failed";
    return false;
  }
  AINFO << "Lidar Detection Filter Component Configs: "
        << comp_config.DebugString();

  output_channel_name_ = comp_config.output_channel_name();
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

  use_object_filter_bank_ = comp_config.use_object_filter_bank();
  if (use_object_filter_bank_) {
    auto plugin_param = comp_config.plugin_param();
    ObjectFilterInitOptions filter_bank_init_options;
    filter_bank_init_options.config_path = plugin_param.config_path();
    filter_bank_init_options.config_file = plugin_param.config_file();
    ACHECK(filter_bank_.Init(filter_bank_init_options));
  }

  return true;
}

bool LidarDetectionFilterComponent::Proc(
    const std::shared_ptr<LidarFrameMessage>& message) {
  PERF_FUNCTION()
  // internal proc
  bool status = InternalProc(message);
  if (status) {
    writer_->Write(message);
    AINFO << "Send lidar detection filter message.";
  }
  return status;
}

bool LidarDetectionFilterComponent::InternalProc(
    const std::shared_ptr<LidarFrameMessage>& in_message) {
  ObjectFilterOptions filter_options;

  PERF_BLOCK("filter_bank")
  if (use_object_filter_bank_) {
    if (!filter_bank_.Filter(filter_options, in_message->lidar_frame_.get())) {
      AINFO << "Lidar detection filter banck error.";
      return false;
    }
  }
  PERF_BLOCK_END

  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
