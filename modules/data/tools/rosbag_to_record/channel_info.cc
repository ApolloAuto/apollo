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

#include "modules/data/tools/rosbag_to_record/channel_info.h"

#include "modules/transform/proto/transform.pb.h"

namespace apollo {
namespace data {

namespace {
static const char* empty_str_ = "";
}  // namespace

ChannelInfo::ChannelInfo() { InitChannels(); }

ChannelInfo::~ChannelInfo() {
  channel_msg_type_.clear();
  channel_proto_desc_.clear();
}

const std::string ChannelInfo::GetMessageType(const std::string& channel_name) {
  auto search = channel_msg_type_.find(channel_name);
  if (search != channel_msg_type_.end()) {
    return search->second;
  }
  return empty_str_;
}

const std::string ChannelInfo::GetProtoDesc(const std::string& channel_name) {
  auto search = channel_proto_desc_.find(channel_name);
  if (search != channel_proto_desc_.end()) {
    return search->second;
  }
  return empty_str_;
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
  InitChannelInfo<apollo::canbus::Chassis>("/apollo/canbus/chassis",
                                           "apollo.canbus.Chassis");
  InitChannelInfo<apollo::control::ControlCommand>(
      "/apollo/control", "apollo.control.ControlCommand");
  InitChannelInfo<apollo::guardian::GuardianCommand>(
      "/apollo/guardian", "apollo.guardian.GuardianCommand");
  InitChannelInfo<apollo::localization::LocalizationEstimate>(
      "/apollo/localization/pose", "apollo.localization.LocalizationEstimate");
  InitChannelInfo<apollo::perception::TrafficLightDetection>(
      "/apollo/perception/traffic_light",
      "apollo.perception.TrafficLightDetection");
  InitChannelInfo<apollo::common::DriveEvent>("/apollo/drive_event",
                                              "apollo.common.DriveEvent");
  InitChannelInfo<apollo::localization::Gps>("/apollo/sensor/gnss/odometry",
                                             "apollo.localization.Gps");
  InitChannelInfo<apollo::data::StaticInfo>("/apollo/monitor/static_info",
                                            "apollo.data.StaticInfo");
  InitChannelInfo<apollo::common::monitor::MonitorMessage>(
      "/apollo/monitor", "apollo.common.monitor.MonitorMessage");

  InitChannelInfo<apollo::canbus::ChassisDetail>(
      "/apollo/canbus/chassis_detail", "apollo.canbus.ChassisDetail");
  InitChannelInfo<apollo::control::PadMessage>("/apollo/control/pad",
                                               "apollo.control.PadMessage");
  InitChannelInfo<apollo::relative_map::NavigationInfo>(
      "/apollo/navigation", "apollo.relative_map.NavigationInfo");
  InitChannelInfo<apollo::routing::RoutingRequest>(
      "/apollo/routing_request", "apollo.routing.RoutingRequest");
  InitChannelInfo<apollo::routing::RoutingResponse>(
      "/apollo/routing_response", "apollo.routing.RoutingResponse");
  InitChannelInfo<apollo::transform::TransformStampeds>(
      "/tf", "apollo.transform.TransformStampeds");
  InitChannelInfo<apollo::transform::TransformStampeds>(
      "/tf_static", "apollo.transform.TransformStampeds");
  InitChannelInfo<apollo::drivers::ContiRadar>("/apollo/sensor/conti_radar",
                                               "apollo.drivers.ContiRadar");
  InitChannelInfo<apollo::drivers::DelphiESR>("/apollo/sensor/delphi_esr",
                                              "apollo.drivers.DelphiESR");
  InitChannelInfo<apollo::drivers::gnss::GnssBestPose>(
      "/apollo/sensor/gnss/best_pose", "apollo.drivers.gnss.GnssBestPose");
  InitChannelInfo<apollo::drivers::gnss::Imu>("/apollo/sensor/gnss/imu",
                                              "apollo.drivers.gnss.Imu");
  InitChannelInfo<apollo::drivers::gnss::InsStat>(
      "/apollo/sensor/gnss/ins_stat", "apollo.drivers.gnss.InsStat");
  InitChannelInfo<apollo::drivers::gnss::GnssEphemeris>(
      "/apollo/sensor/gnss/rtk_eph", "apollo.drivers.gnss.GnssEphemeris");
  InitChannelInfo<apollo::drivers::gnss::EpochObservation>(
      "/apollo/sensor/gnss/rtk_obs", "apollo.drivers.gnss.EpochObservation");
  InitChannelInfo<apollo::drivers::PointCloud>(
      "/apollo/sensor/velodyne64/compensator/PointCloud2",
      "apollo.drivers.PointCloud");
}

}  // namespace data
}  // namespace apollo
