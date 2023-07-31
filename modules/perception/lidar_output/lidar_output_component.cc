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

#include "modules/perception/lidar_output/lidar_output_component.h"

#include <vector>

#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/common/onboard/msg_serializer/msg_serializer.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::SetProtoToASCIIFile;

bool LidarOutputComponent::Init() {
  LidarOutputComponentConfig comp_config;
  if (!GetProtoConfig(&comp_config)) {
    AERROR << "Get LidarOutputComponentConfig file failed";
    return false;
  }
  AINFO << "Lidar Output Component Configs: "
        << comp_config.DebugString();

  // writer
  output_channel_name_ = comp_config.output_channel_name();
  writer_ = node_->CreateWriter<PerceptionObstacles>(output_channel_name_);

  return true;
}

bool LidarOutputComponent::Proc(
    const std::shared_ptr<SensorFrameMessage>& in_message) {
  std::shared_ptr<PerceptionObstacles> out_message(new (std::nothrow)
                                                       PerceptionObstacles);
  double timestamp = in_message->timestamp_;
  double lidar_timestamp = in_message->lidar_timestamp_;
  apollo::common::ErrorCode error_code = apollo::common::ErrorCode::OK;
  std::vector<base::ObjectPtr> valid_objects;

  if (in_message != nullptr && in_message->frame_ != nullptr) {
    auto lidar_objects = in_message->frame_->objects;
    valid_objects.assign(lidar_objects.begin(), lidar_objects.end());
  }

  if (!onboard::MsgSerializer::SerializeMsg(
          timestamp,
          lidar_timestamp,
          in_message->seq_num_,
          valid_objects,
          error_code,
          out_message.get())) {
    AERROR << "Failed to gen PerceptionObstacles object.";
    return false;
  }

  // save proto message to file
  SaveBenchmarkFrame(in_message);
  // // save proto message to file
  // std::string pb_name = "/apollo/data/benchmark/bp/" +
  //                       std::to_string(timestamp) + ".pb";
  // SetProtoToASCIIFile(*out_message, pb_name);

  writer_->Write(out_message);
  AINFO << "Send lidar tracking output message.";

  return true;
}

bool LidarOutputComponent::SaveBenchmarkFrame(
    const std::shared_ptr<SensorFrameMessage>& in_message) {
  std::shared_ptr<PerceptionBenchmarkFrame> out_message(new (std::nothrow)
      PerceptionBenchmarkFrame);
  double timestamp = in_message->timestamp_;
  double lidar_timestamp = in_message->lidar_timestamp_;
  std::vector<base::ObjectPtr> valid_objects;

  if (in_message != nullptr && in_message->frame_ != nullptr) {
    auto lidar_objects = in_message->frame_->objects;
    valid_objects.assign(lidar_objects.begin(), lidar_objects.end());
  }

  auto sensor_frame_info = out_message->mutable_sensor_frame_info();
  sensor_frame_info->set_sensor_id(in_message->sensor_id_);
  sensor_frame_info->set_timestamp(timestamp);

  Eigen::Affine3d sensor2world_pose = Eigen::Affine3d::Identity();
  if (in_message == nullptr || in_message->frame_ == nullptr) {
    AERROR << "SaveBenchmarkFrame, Can't get sensor2world_pose.";
  }
  sensor2world_pose = in_message->frame_->sensor2world_pose;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double val = sensor2world_pose.matrix()(i, j);
      sensor_frame_info->add_sensor2world_pose(val);
    }
  }

  // Serialize benchmark message
  if (!onboard::MsgSerializer::SerializeBenchmarkMsg(
      timestamp,
      lidar_timestamp,
      in_message->seq_num_,
      valid_objects,
      out_message.get())) {
    AERROR << "Failed to get PerceptionBenchmarkFrame object.";
    return false;
  }
  std::string pb_name = "/apollo/data/benchmark/dkit/pb/" +
                        std::to_string(timestamp) + ".pb";
  // SetProtoToASCIIFile(*out_message, pb_name);

  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
