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
#include <gflags/gflags.h>
#include "modules/perception/lib/utils/perf.h"
#include "modules/perception/lib/utils/time_util.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/onboard/common_flags/common_flags.h"
#include "modules/perception/onboard/component/lidar_common_flags.h"

namespace apollo {
namespace perception {
namespace onboard {

uint32_t SegmentationComponent::s_seq_num_ = 0;
std::mutex SegmentationComponent::s_mutex_;

bool SegmentationComponent::Init() {
  if (InitAlgorithmPlugin() != true) {
    AERROR << "Failed to init segmentation component algorithm plugin.";
    return false;
  }
  writer_ = node_->CreateWriter<LidarFrameMessage>(
      FLAGS_obs_segmentation_component_output_channel_name);
  return true;
}

bool SegmentationComponent::Proc(
    const std::shared_ptr<drivers::PointCloud>& message) {
  AINFO << "Enter segmentation component, message timestamp: "
        << GLOG_TIMESTAMP(message->measurement_time()) << " current timestamp "
        << GLOG_TIMESTAMP(lib::TimeUtil::GetCurrentTime());

  std::shared_ptr<LidarFrameMessage> out_message(new (std::nothrow)
                                                     LidarFrameMessage);

  bool status = InternalProc(message, out_message);
  if (status == true) {
    writer_->Write(out_message);
    // Send(FLAGS_obs_segmentation_component_output_channel_name, out_message);
    AINFO << "Send lidar segment output message.";
  }
  return status;
}

bool SegmentationComponent::InitAlgorithmPlugin() {
  segmentor_.reset(new lidar::LidarObstacleSegmentation);
  if (segmentor_ == nullptr) {
    AERROR << "Failed to get segmentation instance";
    return false;
  }
  lidar::LidarObstacleSegmentationInitOptions init_options;
  init_options.enable_hdmap_input = FLAGS_obs_enable_hdmap_input;
  if (!segmentor_->Init(init_options)) {
    AINFO << "Failed to init segmentation.";
    return false;
  }
  velodyne2world_trans_.Init(FLAGS_obs_lidar2novatel_tf2_child_frame_id);
  return true;
}

bool SegmentationComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud>& in_message,
    const std::shared_ptr<LidarFrameMessage>& out_message) {
  PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(FLAGS_obs_lidar_onboard_sensor_name);
  {
    std::unique_lock<std::mutex> lock(s_mutex_);
    s_seq_num_++;
  }
  const double timestamp = in_message->measurement_time();
  const double cur_time = lib::TimeUtil::GetCurrentTime();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << "FRAME_STATISTICS:Lidar:Start:msg_time[" << GLOG_TIMESTAMP(timestamp)
        << "]:cur_time[" << GLOG_TIMESTAMP(cur_time) << "]:cur_latency["
        << start_latency << "]";

  out_message->timestamp_ = timestamp;
  out_message->seq_num_ = s_seq_num_;
  out_message->process_stage_ = ProcessStage::LIDAR_SEGMENTATION;
  out_message->error_code_ = apollo::common::ErrorCode::OK;

  PERCEPTION_PERF_BLOCK_START();
  Eigen::Affine3d velodyne_trans;
  const double velodyne64_query_tf_timestamp =
      timestamp - FLAGS_obs_velodyne64_query_tf_offset * 0.001;
  if (velodyne2world_trans_.GetSensor2worldTrans(velodyne64_query_tf_timestamp,
                                                 &velodyne_trans) != true) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Fail to get pose at time: "
           << GLOG_TIMESTAMP(velodyne64_query_tf_timestamp);
    return true;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(
      FLAGS_obs_lidar_onboard_sensor_name,
      "segmentation_1::get_lidar_to_world_pose");

  auto& frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->lidar2world_pose = velodyne_trans;
  frame->cloud = base::PointFCloudPool::Instance().Get();
  frame->timestamp = timestamp;

#ifdef PERCEPTION_LIDAR_USE_COMMON_MESSAGE
  lidar::LidarObstacleSegmentationOptions segment_opts;
  segment_opts.sensor_name = FLAGS_obs_lidar_onboard_sensor_name;
  lidar::LidarProcessResult ret =
      segmentor_->Process(segment_opts, in_message, frame.get());
  if (ret.error_code != lidar::LidarErrorCode::Succeed) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Lidar segmentation process error, " << ret.log;
  }
  PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(FLAGS_obs_lidar_onboard_sensor_name,
                                           "segmentation_2::segment_obstacle");
#endif

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
