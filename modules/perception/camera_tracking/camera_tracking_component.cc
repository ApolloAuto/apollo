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

#include "modules/perception/camera_tracking/camera_tracking_component.h"

#include "modules/perception/camera_tracking/proto/camera_tracking_component.pb.h"

#include "cyber/common/file.h"
#include "cyber/profiler/profiler.h"
#include "cyber/time/clock.h"

namespace apollo {
namespace perception {
namespace camera {

using Clock = apollo::cyber::Clock;

bool CameraTrackingComponent::Init() {
  CameraTrackingComponentConfig camera_tracking_component_config;
  if (!GetProtoConfig(&camera_tracking_component_config)) {
    return false;
  }
  AINFO << "Camera Tracking Component config: "
        << camera_tracking_component_config.DebugString();
  output_channel_name_ = camera_tracking_component_config.output_channel_name();
  writer_ =
      node_->CreateWriter<onboard::SensorFrameMessage>(output_channel_name_);

  sensor_manager_ = algorithm::SensorManager::Instance();

  PluginParam plugin_param = camera_tracking_component_config.plugin_param();
  std::string tracker_name = plugin_param.name();
  camera_obstacle_tracker_.reset(
      BaseObstacleTrackerRegisterer::GetInstanceByName(tracker_name));
  if (nullptr == camera_obstacle_tracker_) {
    AERROR << "Failed to get camera obstacle tracker instance.";
    return false;
  }

  ObstacleTrackerInitOptions tracker_init_options;
  tracker_init_options.config_path = plugin_param.config_path();
  tracker_init_options.config_file = plugin_param.config_file();
  tracker_init_options.image_width =
      camera_tracking_component_config.image_width();
  tracker_init_options.image_height =
      camera_tracking_component_config.image_height();
  tracker_init_options.gpu_id = camera_tracking_component_config.gpu_id();
  if (!camera_obstacle_tracker_->Init(tracker_init_options)) {
    AERROR << "Failed to init camera obstacle tracker.";
    return false;
  }
  return true;
}

bool CameraTrackingComponent::Proc(
    const std::shared_ptr<onboard::CameraFrame>& message) {
  PERF_FUNCTION()
  AINFO << std::setprecision(16)
        << "Enter Tracking component, message timestamp: " << message->timestamp
        << " current timestamp: " << Clock::NowInSeconds();

  auto out_message = std::make_shared<onboard::SensorFrameMessage>();
  bool status = InternalProc(message, out_message);

  if (status) {
    writer_->Write(out_message);
    AINFO << "Send camera tracking output message.";
  }
  return status;
}

bool CameraTrackingComponent::InternalProc(
    const std::shared_ptr<const onboard::CameraFrame>& in_message,
    std::shared_ptr<onboard::SensorFrameMessage> out_message) {
  out_message->timestamp_ = in_message->timestamp;
  std::shared_ptr<CameraTrackingFrame> tracking_frame;
  tracking_frame.reset(new CameraTrackingFrame);
  tracking_frame->frame_id = in_message->frame_id;
  tracking_frame->timestamp = in_message->timestamp;
  tracking_frame->data_provider = in_message->data_provider;
  tracking_frame->detected_objects = in_message->detected_objects;
  tracking_frame->feature_blob = in_message->feature_blob;
  tracking_frame->track_feature_blob.reset(new base::Blob<float>());
  tracking_frame->project_matrix.setIdentity();
  tracking_frame->camera2world_pose = in_message->camera2world_pose;

  // Tracking
  PERF_BLOCK("camera_tracking")
  bool res = camera_obstacle_tracker_->Process(tracking_frame);
  PERF_BLOCK_END

  if (!res) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Camera tracking process error!";
    return false;
  }

  base::SensorInfo sensor_info;
  std::string camera_name = in_message->data_provider->sensor_name();
  if (!(sensor_manager_->GetSensorInfo(camera_name, &sensor_info))) {
    AERROR << "Failed to get sensor info, sensor name: " << camera_name;
    return false;
  }

  auto& frame = out_message->frame_;
  frame = base::FramePool::Instance().Get();
  // todo(wxt): check if sensor_info needed
  // frame->sensor_info = lidar_frame->sensor_info;
  frame->sensor_info = sensor_info;
  frame->timestamp = in_message->timestamp;
  frame->objects = tracking_frame->tracked_objects;
  // todo(wxt): check if sensor2world_pose needed
  frame->sensor2world_pose = tracking_frame->camera2world_pose;
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
