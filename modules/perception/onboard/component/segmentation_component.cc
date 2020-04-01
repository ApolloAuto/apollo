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
#include "modules/perception/onboard/component/segmentation_component.h"

#include "modules/common/time/time.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lib/utils/perf.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/onboard/common_flags/common_flags.h"

namespace apollo {
namespace perception {
namespace onboard {

uint32_t SegmentationComponent::s_seq_num_ = 0;
std::mutex SegmentationComponent::s_mutex_;

bool SegmentationComponent::Init() {
  LidarSegmentationComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    return false;
  }
  ADEBUG << "Lidar Component Configs: " << comp_config.DebugString();
  output_channel_name_ = comp_config.output_channel_name();
  sensor_name_ = comp_config.sensor_name();
  lidar2novatel_tf2_child_frame_id_ =
      comp_config.lidar2novatel_tf2_child_frame_id();
  lidar_query_tf_offset_ =
      static_cast<float>(comp_config.lidar_query_tf_offset());
  enable_hdmap_ = comp_config.enable_hdmap();
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init segmentation component algorithm plugin.";
    return false;
  }
  return true;
}

bool SegmentationComponent::Proc(
    const std::shared_ptr<drivers::PointCloud>& message) {
  AINFO << "Enter segmentation component, message timestamp: "
        << message->measurement_time() << " current timestamp: "
        << apollo::common::time::Clock::NowInSeconds();

  std::shared_ptr<LidarFrameMessage> out_message(new (std::nothrow)
                                                     LidarFrameMessage);

  bool status = InternalProc(message, out_message);
  if (status) {
    writer_->Write(out_message);
    AINFO << "Send lidar segment output message.";
  }
  return status;
}

bool SegmentationComponent::InitAlgorithmPlugin() {
  ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_,
                                                          &sensor_info_));

  segmentor_.reset(new lidar::LidarObstacleSegmentation);
  if (segmentor_ == nullptr) {
    AERROR << "sensor_name_ "
           << "Failed to get segmentation instance";
    return false;
  }
  lidar::LidarObstacleSegmentationInitOptions init_options;
  init_options.sensor_name = sensor_name_;
  init_options.enable_hdmap_input =
      FLAGS_obs_enable_hdmap_input && enable_hdmap_;
  if (!segmentor_->Init(init_options)) {
    AINFO << "sensor_name_ "
          << "Failed to init segmentation.";
    return false;
  }

  lidar2world_trans_.Init(lidar2novatel_tf2_child_frame_id_);
  return true;
}

bool SegmentationComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud>& in_message,
    const std::shared_ptr<LidarFrameMessage>& out_message) {
  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(sensor_name_);
  {
    std::unique_lock<std::mutex> lock(s_mutex_);
    s_seq_num_++;
  }
  const double timestamp = in_message->measurement_time();
  const double cur_time = apollo::common::time::Clock::NowInSeconds();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Lidar:Start:msg_time[" << timestamp << sensor_name_
        << ":Start:msg_time["
        << "]:cur_time[" << cur_time << "]:cur_latency[" << start_latency
        << "]";

  out_message->timestamp_ = timestamp;
  out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
  out_message->seq_num_ = s_seq_num_;
  out_message->process_stage_ = ProcessStage::LIDAR_SEGMENTATION;
  out_message->error_code_ = apollo::common::ErrorCode::OK;

  auto& frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->cloud = base::PointFCloudPool::Instance().Get();
  frame->timestamp = timestamp;
  frame->sensor_info = sensor_info_;

  PERCEPTION_PERF_BLOCK_START();
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose_novatel = Eigen::Affine3d::Identity();
  const double lidar_query_tf_timestamp =
      timestamp - lidar_query_tf_offset_ * 0.001;
  if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp, &pose,
                                               &pose_novatel)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get pose at time: " << lidar_query_tf_timestamp;
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
      sensor_name_, "segmentation_1::get_lidar_to_world_pose");

  frame->lidar2world_pose = pose;
  frame->novatel2world_pose = pose_novatel;

  lidar::LidarObstacleSegmentationOptions segment_opts;
  segment_opts.sensor_name = sensor_name_;
  lidar2world_trans_.GetExtrinsics(&segment_opts.sensor2novatel_extrinsics);
  lidar::LidarProcessResult ret =
      segmentor_->Process(segment_opts, in_message, frame.get());
  if (ret.error_code != lidar::LidarErrorCode::Succeed) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Lidar segmentation process error, " << ret.log;
    return false;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(sensor_name_,
                                           "segmentation_2::segment_obstacle");

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
