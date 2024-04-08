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

#include "modules/perception/lidar_tracking/lidar_tracking_component.h"

#include "cyber/profiler/profiler.h"
#include "cyber/time/clock.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::Clock;

bool LidarTrackingComponent::Init() {
  LidarTrackingComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get LidarTrackingComponentConfig file failed";
    return false;
  }
  AINFO << "Lidar Tracking Component Configs: " << comp_config.DebugString();

  // writer
  std::string output_channel_name = comp_config.output_channel_name();
  writer_ = node_->CreateWriter<SensorFrameMessage>(output_channel_name);

  // multi target tracking init
  auto multi_target_tracker_param = comp_config.multi_target_tracker_param();
  std::string multi_target_tracker_name = multi_target_tracker_param.name();
  multi_target_tracker_ = BaseMultiTargetTrackerRegisterer::GetInstanceByName(
      multi_target_tracker_name);
  CHECK_NOTNULL(multi_target_tracker_);

  MultiTargetTrackerInitOptions tracker_init_options;
  tracker_init_options.config_path = multi_target_tracker_param.config_path();
  tracker_init_options.config_file = multi_target_tracker_param.config_file();
  ACHECK(multi_target_tracker_->Init(tracker_init_options));

  // // fused classifier init
  // auto fusion_classifier_param = comp_config.fusion_classifier_param();
  // std::string fusion_classifier_name = fusion_classifier_param.name();
  // fusion_classifier_ =
  //     BaseClassifierRegisterer::GetInstanceByName(fusion_classifier_name);
  // CHECK_NOTNULL(fusion_classifier_);

  // ClassifierInitOptions fusion_classifier_init_options;
  // fusion_classifier_init_options.config_path =
  //     fusion_classifier_param.config_path();
  // fusion_classifier_init_options.config_file =
  //     fusion_classifier_param.config_file();
  // ACHECK(fusion_classifier_->Init(fusion_classifier_init_options));
  return true;
}

bool LidarTrackingComponent::Proc(
    const std::shared_ptr<LidarFrameMessage>& message) {
  PERF_FUNCTION()
  AINFO << std::setprecision(16)
        << "Enter LidarTracking component, message timestamp: "
        << message->timestamp_
        << " current timestamp: " << Clock::NowInSeconds();

  auto out_message = std::make_shared<SensorFrameMessage>();

  if (InternalProc(message, out_message)) {
    writer_->Write(out_message);
    return true;
  }

  AERROR << "Send lidar tracking output message failed!";
  return false;
}

bool LidarTrackingComponent::InternalProc(
    const std::shared_ptr<const LidarFrameMessage>& in_message,
    const std::shared_ptr<SensorFrameMessage>& out_message) {
  out_message->timestamp_ = in_message->timestamp_;
  out_message->lidar_timestamp_ = in_message->lidar_timestamp_;
  out_message->seq_num_ = in_message->seq_num_;
  out_message->process_stage_ = onboard::ProcessStage::LIDAR_RECOGNITION;
  out_message->sensor_id_ = in_message->lidar_frame_->sensor_info.name;

  if (in_message->error_code_ != apollo::common::ErrorCode::OK) {
    out_message->error_code_ = in_message->error_code_;
    AERROR << "Lidar tracking receive message with error code, skip it.";
    return true;
  }

  auto& lidar_frame = in_message->lidar_frame_;
  // multi target tracker
  PERF_BLOCK("multi_target_tracker")
  MultiTargetTrackerOptions tracker_options;
  if (!multi_target_tracker_->Track(tracker_options, lidar_frame.get())) {
    AINFO << "Lidar tracking, multi_target_tracker_ Track error.";
    return false;
  }
  PERF_BLOCK_END

  // // fused classifer
  // PERF_BLOCK("fusion_classifier")
  // ClassifierOptions fusion_classifier_options;
  // if (!fusion_classifier_->Classify(fusion_classifier_options,
  //                                   lidar_frame.get())) {
  //   AERROR << "Lidar tracking, fusion_classifier_ Classify error.";
  //   return false;
  // }
  // PERF_BLOCK_END

  // get out_message
  out_message->hdmap_ = lidar_frame->hdmap_struct;
  auto& frame = out_message->frame_;
  frame = base::FramePool::Instance().Get();
  frame->sensor_info = lidar_frame->sensor_info;
  frame->timestamp = in_message->timestamp_;
  frame->objects = lidar_frame->tracked_objects;
  frame->sensor2world_pose = lidar_frame->lidar2world_pose;
  frame->lidar_frame_supplement.on_use = true;
  frame->lidar_frame_supplement.cloud_ptr = lidar_frame->cloud;

  const double end_timestamp = Clock::NowInSeconds();
  const double end_latency = (end_timestamp - in_message->timestamp_) * 1e3;
  AINFO << std::setprecision(16)
        << "FRAME_STATISTICS:LidarTracking:End:msg_time["
        << in_message->timestamp_ << "]:cur_time[" << end_timestamp
        << "]:cur_latency[" << end_latency << "]";

  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
