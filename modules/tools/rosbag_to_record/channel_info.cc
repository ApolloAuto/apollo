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

#include "modules/tools/rosbag_to_record/channel_info.h"

namespace apollo {
namespace tools {

static const std::string DUMMY_STRING = "";

ChannelInfo::ChannelInfo() {
  InitChannels();
}

ChannelInfo::~ChannelInfo() {
  channel_msg_type_.clear();
  channel_proto_desc_.clear();
}

const std::string& ChannelInfo::GetMessageType(
    const std::string& channel_name) {
  auto search = channel_msg_type_.find(channel_name);
  if (search != channel_msg_type_.end()) {
    return search->second;
  }
  return DUMMY_STRING;
}

const std::string& ChannelInfo::GetProtoDesc(const std::string& channel_name) {
  auto search = channel_proto_desc_.find(channel_name);
  if (search != channel_proto_desc_.end()) {
    return search->second;
  }
  return DUMMY_STRING;
}

const std::vector<std::string>& ChannelInfo::GetSupportChannels() {
  return support_channels_;
}

void ChannelInfo::InitChannels() {
  InitChannelInfo<apollo::perception::PerceptionObstacles>(
      "/apollo/perception/obstacles", "apollo.perception.PerceptionObstacles");
  InitChannelInfo<apollo::planning::ADCTrajectory>(
      "/apollo/planning", "apollo.planning.ADCTrajectory");
  InitChannelInfo<apollo::prediction::PredictionObstacles>(
      "/apollo/prediction", "apollo.prediction.PredictionObstacles");
  InitChannelInfo<apollo::canbus::Chassis>(
      "/apollo/canbus/chassis", "apollo.canbus.Chassis");
  InitChannelInfo<apollo::control::ControlCommand>(
      "/apollo/control", "apollo.control.ControlCommand");
  InitChannelInfo<apollo::guardian::GuardianCommand>(
      "/apollo/guardian", "apollo.guardian.GuardianCommand");
  InitChannelInfo<apollo::localization::LocalizationEstimate>(
      "/apollo/localization/pose", "apollo.localization.LocalizationEstimate");
  InitChannelInfo<apollo::perception::TrafficLightDetection>(
      "/apollo/perception/traffic_light", "apollo.perception.TrafficLightDetection");
  InitChannelInfo<apollo::common::DriveEvent>(
      "/apollo/drive_event", "apollo.common.DriveEvent");
  InitChannelInfo<apollo::localization::CorrectedImu>(
      "/apollo/sensor/gnss/corrected_imu", "apollo.localization.CorrectedImu");
  InitChannelInfo<apollo::localization::Gps>(
      "/apollo/sensor/gnss/odometry", "apollo.localization.Gps");
}

}  // namespace tools
}  // namespace apollo
