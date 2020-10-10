/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/onboard/component/detection_component.h"

#include "cyber/time/clock.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/onboard/common_flags/common_flags.h"

using ::apollo::cyber::Clock;

namespace apollo {
namespace perception {
namespace onboard {

std::atomic<uint32_t> DetectionComponent::seq_num_{0};

bool DetectionComponent::Init() {
  LidarDetectionComponentConfig comp_config;
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
  //  enable_hdmap_ = comp_config.enable_hdmap();
  writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);

  if (!InitAlgorithmPlugin()) {
    AERROR << "Failed to init detection component algorithm plugin.";
    return false;
  }
  return true;
}

bool DetectionComponent::Proc(
    const std::shared_ptr<drivers::PointCloud>& message) {
  AINFO << std::setprecision(16)
        << "Enter detection component, message timestamp: "
        << message->measurement_time()
        << " current timestamp: " << Clock::NowInSeconds();

  auto out_message = std::make_shared<LidarFrameMessage>();

  bool status = InternalProc(message, out_message);
  if (status) {
    writer_->Write(out_message);
    AINFO << "Send lidar detect output message.";
  }
  return status;
}

bool DetectionComponent::InitAlgorithmPlugin() {
  ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_,
                                                          &sensor_info_));

  detector_.reset(new lidar::LidarObstacleDetection);
  if (detector_ == nullptr) {
    AERROR << "sensor_name_ "
           << "Failed to get detection instance";
    return false;
  }
  lidar::LidarObstacleDetectionInitOptions init_options;
  init_options.sensor_name = sensor_name_;
  //  init_options.enable_hdmap_input =
  //      FLAGS_obs_enable_hdmap_input && enable_hdmap_;
  if (!detector_->Init(init_options)) {
    AINFO << "sensor_name_ "
          << "Failed to init detection.";
    return false;
  }

  lidar2world_trans_.Init(lidar2novatel_tf2_child_frame_id_);
  return true;
}

bool DetectionComponent::InternalProc(
    const std::shared_ptr<const drivers::PointCloud>& in_message,
    const std::shared_ptr<LidarFrameMessage>& out_message) {
  uint32_t seq_num = seq_num_.fetch_add(1);
  const double timestamp = in_message->measurement_time();
  const double cur_time = Clock::NowInSeconds();
  const double start_latency = (cur_time - timestamp) * 1e3;
  AINFO << std::setprecision(16) << "FRAME_STATISTICS:Lidar:Start:msg_time["
        << timestamp << "]:sensor[" << sensor_name_ << "]:cur_time[" << cur_time
        << "]:cur_latency[" << start_latency << "]";

  out_message->timestamp_ = timestamp;
  out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
  out_message->seq_num_ = seq_num;
  out_message->process_stage_ = ProcessStage::LIDAR_DETECTION;
  out_message->error_code_ = apollo::common::ErrorCode::OK;

  auto& frame = out_message->lidar_frame_;
  frame = lidar::LidarFramePool::Instance().Get();
  frame->cloud = base::PointFCloudPool::Instance().Get();
  frame->timestamp = timestamp;
  frame->sensor_info = sensor_info_;

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  const double lidar_query_tf_timestamp =
      timestamp - lidar_query_tf_offset_ * 0.001;
  if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp,
                                               &pose)) {
    out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
    AERROR << "Failed to get pose at time: "
           << FORMAT_TIMESTAMP(lidar_query_tf_timestamp);
    return false;
  }

  frame->lidar2world_pose = pose;

  lidar::LidarObstacleDetectionOptions detect_opts;
  detect_opts.sensor_name = sensor_name_;
  lidar2world_trans_.GetExtrinsics(&detect_opts.sensor2novatel_extrinsics);
  lidar::LidarProcessResult ret =
      detector_->Process(detect_opts, in_message, frame.get());
  if (ret.error_code != lidar::LidarErrorCode::Succeed) {
    out_message->error_code_ =
        apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
    AERROR << "Lidar detection process error, " << ret.log;
    return false;
  }

  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
