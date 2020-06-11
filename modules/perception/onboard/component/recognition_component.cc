/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/onboard/component/recognition_component.h"
#include "modules/common/time/time.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/utils/perf.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_log.h"
// #include "modules/perception/onboard/component/lidar_common_flags.h"

namespace apollo {
namespace perception {
namespace onboard {

bool RecognitionComponent::Init() {
  LidarRecognitionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Lidar Component Configs: " << comp_config.DebugString();
  output_channel_name_ = comp_config.output_channel_name();
  main_sensor_name_ = comp_config.main_sensor_name();
  writer_ = node_->CreateWriter<SensorFrameMessage>(output_channel_name_);
  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init recongnition component algorithm plugin.";
    return false;
  }
  return true;
}

bool RecognitionComponent::Proc(
    const std::shared_ptr<LidarFrameMessage>& message) {
  AINFO << "Enter Tracking component, message timestamp: "
        << message->timestamp_ << " current timestamp: "
        << apollo::common::time::Clock::NowInSeconds();

  std::shared_ptr<SensorFrameMessage> out_message =
      std::make_shared<SensorFrameMessage>();

  if (InternalProc(message, out_message)) {
    writer_->Write(out_message);
    AINFO << "Send lidar recognition output message.";
    return true;
  }
  return false;
}

bool RecognitionComponent::InitAlgorithmPlugin() {
  tracker_.reset(new lidar::LidarObstacleTracking);
  if (tracker_ == nullptr) {
    AERROR << "Failed to get tracking instance.";
    return false;
  }
  lidar::LidarObstacleTrackingInitOptions init_options;
  init_options.sensor_name = main_sensor_name_;
  if (!tracker_->Init(init_options)) {
    AERROR << "Failed to init tracking.";
    return false;
  }

  return true;
}

bool RecognitionComponent::InternalProc(
    const std::shared_ptr<const LidarFrameMessage>& in_message,
    const std::shared_ptr<SensorFrameMessage>& out_message) {
  auto& sensor_name = in_message->lidar_frame_->sensor_info.name;
  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(sensor_name);
  out_message->timestamp_ = in_message->timestamp_;
  out_message->lidar_timestamp_ = in_message->lidar_timestamp_;
  out_message->seq_num_ = in_message->seq_num_;
  out_message->process_stage_ = ProcessStage::LIDAR_RECOGNITION;
  out_message->sensor_id_ = sensor_name;

  if (in_message->error_code_ != apollo::common::ErrorCode::OK) {
    out_message->error_code_ = in_message->error_code_;
    AERROR << "Lidar recognition receive message with error code, skip it.";
    return true;
  }

  PERCEPTION_PERF_BLOCK_START();
  auto& lidar_frame = in_message->lidar_frame_;
  lidar::LidarObstacleTrackingOptions track_options;
  track_options.sensor_name = sensor_name;
  lidar::LidarProcessResult ret =
      tracker_->Process(track_options, lidar_frame.get());
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name,
                                           "recognition_1::track_obstacle");
  if (ret.error_code != lidar::LidarErrorCode::Succeed) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Lidar recognition process error, " << ret.log;
    return true;
  }
  // TODO(shigintmin)
  out_message->hdmap_ = lidar_frame->hdmap_struct;
  auto& frame = out_message->frame_;
  frame = base::FramePool::Instance().Get();
  frame->sensor_info = lidar_frame->sensor_info;
  frame->timestamp = in_message->timestamp_;
  frame->objects = lidar_frame->tracked_objects;
  frame->sensor2world_pose = lidar_frame->lidar2world_pose;
  frame->lidar_frame_supplement.on_use = true;
  frame->lidar_frame_supplement.cloud_ptr = lidar_frame->cloud;
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name,
                                           "recognition_2::fill_out_message");

  const double end_timestamp = apollo::common::time::Clock::NowInSeconds();
  const double end_latency = (end_timestamp - in_message->timestamp_) * 1e3;
  AINFO << "FRAME_STATISTICS:Lidar:End:msg_time[" << in_message->timestamp_
        << "]:cur_time[" << end_timestamp << "]:cur_latency[" << end_latency
        << "]";
  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
