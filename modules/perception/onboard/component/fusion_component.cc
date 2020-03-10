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
#include "modules/perception/onboard/component/fusion_component.h"

#include "modules/common/time/time.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/lib/utils/perf.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/msg_serializer/msg_serializer.h"

namespace apollo {
namespace perception {
namespace onboard {

uint32_t FusionComponent::s_seq_num_ = 0;
std::mutex FusionComponent::s_mutex_;

bool FusionComponent::Init() {
  FusionComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  AINFO << "Fusion Component Configs: " << comp_config.DebugString();

  // to load component configs
  fusion_method_ = comp_config.fusion_method();
  fusion_main_sensor_ = comp_config.fusion_main_sensor();
  object_in_roi_check_ = comp_config.object_in_roi_check();
  radius_for_roi_object_check_ = comp_config.radius_for_roi_object_check();

  // init algorithm plugin
  ACHECK(InitAlgorithmPlugin()) << "Failed to init algorithm plugin.";
  writer_ = node_->CreateWriter<PerceptionObstacles>(
      comp_config.output_obstacles_channel_name());
  inner_writer_ = node_->CreateWriter<SensorFrameMessage>(
      comp_config.output_viz_fused_content_channel_name());
  return true;
}

bool FusionComponent::Proc(const std::shared_ptr<SensorFrameMessage>& message) {
  if (message->process_stage_ == ProcessStage::SENSOR_FUSION) {
    return true;
  }
  std::shared_ptr<PerceptionObstacles> out_message(new (std::nothrow)
                                                       PerceptionObstacles);
  std::shared_ptr<SensorFrameMessage> viz_message(new (std::nothrow)
                                                      SensorFrameMessage);
  bool status = InternalProc(message, out_message, viz_message);
  if (status) {
    // TODO(conver sensor id)
    if (message->sensor_id_ != fusion_main_sensor_) {
      AINFO << "Fusion receive from " << message->sensor_id_ << "not from "
            << fusion_main_sensor_ << ". Skip send.";
    } else {
      // Send("/apollo/perception/obstacles", out_message);
      writer_->Write(out_message);
      AINFO << "Send fusion processing output message.";
      // send msg for visualization
      if (FLAGS_obs_enable_visualization) {
        // Send("/apollo/perception/inner/PrefusedObjects", viz_message);
        inner_writer_->Write(viz_message);
      }
    }
  }
  return status;
}

bool FusionComponent::InitAlgorithmPlugin() {
  fusion_.reset(new fusion::ObstacleMultiSensorFusion());
  fusion::ObstacleMultiSensorFusionParam param;
  param.main_sensor = fusion_main_sensor_;
  param.fusion_method = fusion_method_;
  ACHECK(fusion_->Init(param)) << "Failed to init ObstacleMultiSensorFusion";

  if (FLAGS_obs_enable_hdmap_input && object_in_roi_check_) {
    hdmap_input_ = map::HDMapInput::Instance();
    ACHECK(hdmap_input_->Init()) << "Failed to init hdmap input.";
  }
  AINFO << "Init algorithm successfully, onboard fusion: " << fusion_method_;
  return true;
}

bool FusionComponent::InternalProc(
    const std::shared_ptr<SensorFrameMessage const>& in_message,
    std::shared_ptr<PerceptionObstacles> out_message,
    std::shared_ptr<SensorFrameMessage> viz_message) {
  {
    std::unique_lock<std::mutex> lock(s_mutex_);
    s_seq_num_++;
  }

  PERCEPTION_PERF_BLOCK_START();
  const double timestamp = in_message->timestamp_;
  const uint64_t lidar_timestamp = in_message->lidar_timestamp_;
  std::vector<base::ObjectPtr> valid_objects;
  if (in_message->error_code_ != apollo::common::ErrorCode::OK) {
    if (!MsgSerializer::SerializeMsg(
            timestamp, lidar_timestamp, in_message->seq_num_, valid_objects,
            in_message->error_code_, out_message.get())) {
      AERROR << "Failed to gen PerceptionObstacles object.";
      return false;
    }
    if (FLAGS_obs_enable_visualization) {
      viz_message->process_stage_ = ProcessStage::SENSOR_FUSION;
      viz_message->error_code_ = in_message->error_code_;
    }
    AERROR << "Fusion receive message with error code, skip it.";
    return true;
  }
  base::FramePtr frame = in_message->frame_;
  frame->timestamp = in_message->timestamp_;

  std::vector<base::ObjectPtr> fused_objects;
  if (!fusion_->Process(frame, &fused_objects)) {
    AERROR << "Failed to call fusion plugin.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(std::string("fusion_process"),
                                           in_message->sensor_id_);

  if (in_message->sensor_id_ != fusion_main_sensor_) {
    return true;
  }

  Eigen::Matrix4d sensor2world_pose =
      in_message->frame_->sensor2world_pose.matrix();
  if (object_in_roi_check_ && FLAGS_obs_enable_hdmap_input) {
    // get hdmap
    base::HdmapStructPtr hdmap(new base::HdmapStruct());
    if (hdmap_input_) {
      base::PointD position;
      position.x = sensor2world_pose(0, 3);
      position.y = sensor2world_pose(1, 3);
      position.z = sensor2world_pose(2, 3);
      hdmap_input_->GetRoiHDMapStruct(position, radius_for_roi_object_check_,
                                      hdmap);
      // TODO(use check)
      // ObjectInRoiSlackCheck(hdmap, fused_objects, &valid_objects);
      valid_objects.assign(fused_objects.begin(), fused_objects.end());
    } else {
      valid_objects.assign(fused_objects.begin(), fused_objects.end());
    }
  } else {
    valid_objects.assign(fused_objects.begin(), fused_objects.end());
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(std::string("fusion_roi_check"),
                                           in_message->sensor_id_);

  // produce visualization msg
  if (FLAGS_obs_enable_visualization) {
    viz_message->timestamp_ = in_message->timestamp_;
    viz_message->seq_num_ = in_message->seq_num_;
    viz_message->frame_ = base::FramePool::Instance().Get();
    viz_message->frame_->sensor2world_pose =
        in_message->frame_->sensor2world_pose;
    viz_message->sensor_id_ = in_message->sensor_id_;
    viz_message->hdmap_ = in_message->hdmap_;
    viz_message->process_stage_ = ProcessStage::SENSOR_FUSION;
    viz_message->error_code_ = in_message->error_code_;
    viz_message->frame_->objects = fused_objects;
  }
  // produce pb output msg
  apollo::common::ErrorCode error_code = apollo::common::ErrorCode::OK;
  if (!MsgSerializer::SerializeMsg(timestamp, lidar_timestamp,
                                   in_message->seq_num_, valid_objects,
                                   error_code, out_message.get())) {
    AERROR << "Failed to gen PerceptionObstacles object.";
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
      std::string("fusion_serialize_message"), in_message->sensor_id_);

  const double cur_time = apollo::common::time::Clock::NowInSeconds();
  const double latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Obstacle:End:msg_time[" << timestamp
        << "]:cur_time[" << cur_time << "]:cur_latency[" << latency
        << "]:obj_cnt[" << valid_objects.size() << "]";
  AINFO << "publish_number: " << valid_objects.size() << " obj";
  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
