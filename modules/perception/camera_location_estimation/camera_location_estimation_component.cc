/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera_location_estimation/camera_location_estimation_component.h"

#include <string>

#include "cyber/profiler/profiler.h"

namespace apollo {
namespace perception {
namespace camera {

void CameraLocationEstimationComponent::InitTransformer(
    const CameraLocationEstimation& location_estimation_param) {
  TransformerInitOptions transformer_init_options;
  auto plugin_param = location_estimation_param.plugin_param();
  transformer_init_options.config_path = plugin_param.config_path();
  transformer_init_options.config_file = plugin_param.config_file();
  transformer_.reset(
      BaseTransformerRegisterer::GetInstanceByName(plugin_param.name()));
  ACHECK(transformer_ != nullptr);
  ACHECK(transformer_->Init(transformer_init_options))
      << "Failed to init: " << plugin_param.name();
}

bool CameraLocationEstimationComponent::Init() {
  CameraLocationEstimation location_estimation_param;
  if (!GetProtoConfig(&location_estimation_param)) {
    AERROR << "Load camera location estimation component config failed!";
    return false;
  }

  InitTransformer(location_estimation_param);

  writer_ = node_->CreateWriter<onboard::CameraFrame>(
      location_estimation_param.channel().output_obstacles_channel_name());
  return true;
}

bool CameraLocationEstimationComponent::Proc(
    const std::shared_ptr<onboard::CameraFrame>& msg) {
  PERF_FUNCTION()
  std::shared_ptr<onboard::CameraFrame> out_message(new (std::nothrow)
                                                        onboard::CameraFrame);
  out_message->frame_id = msg->frame_id;
  out_message->timestamp = msg->timestamp;
  out_message->data_provider = msg->data_provider;
  out_message->detected_objects = msg->detected_objects;
  out_message->feature_blob = msg->feature_blob;
  out_message->camera_k_matrix = msg->camera_k_matrix;
  out_message->camera2world_pose = msg->camera2world_pose;

  PERF_BLOCK("location_estimation_transform")
  if (!transformer_->Transform(out_message.get())) {
    AERROR << "Failed to transform.";
    return false;
  }
  PERF_BLOCK_END

  writer_->Write(out_message);
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
