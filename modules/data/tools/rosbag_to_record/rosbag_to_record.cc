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

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/record/file/record_file_writer.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/data/tools/rosbag_to_record/channel_info.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/transform/proto/transform.pb.h"

using apollo::cyber::proto::SingleMessage;
using apollo::data::ChannelInfo;

void PrintUsage() {
  std::cout << "Usage:\n"
            << "  rosbag_to_record input.bag output.record" << std::endl;
}

int convert_PointCloud(std::shared_ptr<apollo::drivers::PointCloud> proto,
                       sensor_msgs::PointCloud2::ConstPtr rawdata) {
  auto header = proto->mutable_header();
  header->set_timestamp_sec(rawdata->header.stamp.toSec());
  header->set_frame_id(rawdata->header.frame_id);
  header->set_sequence_num(rawdata->header.seq);
  proto->set_frame_id(rawdata->header.frame_id);
  proto->set_measurement_time(rawdata->header.stamp.toSec());
  proto->set_width(rawdata->width);
  proto->set_height(rawdata->height);

  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int stamp_offset = -1;
  int intensity_offset = -1;
  for (const auto &field : rawdata->fields) {
    if (field.name == "x") {
      x_offset = field.offset;
    } else if (field.name == "y") {
      y_offset = field.offset;
    } else if (field.name == "z") {
      z_offset = field.offset;
    } else if (field.name == "timestamp") {
      stamp_offset = field.offset;
    } else if (field.name == "intensity") {
      intensity_offset = field.offset;
    }
  }

  if (x_offset == -1 || y_offset == -1 || z_offset == -1 ||
      stamp_offset == -1 || intensity_offset == -1) {
    std::cerr << "Field not contains x, y, z, timestamp, instensity"
              << std::endl;
    return 0;
  }

  int total = rawdata->width * rawdata->height;
  auto data = rawdata->data;
  for (int i = 0; i < total; ++i) {
    auto cyber_point = proto->add_point();
    int offset = i * rawdata->point_step;
    cyber_point->set_x(*reinterpret_cast<float *>(&data[offset + x_offset]));
    cyber_point->set_y(*reinterpret_cast<float *>(&data[offset + y_offset]));
    cyber_point->set_z(*reinterpret_cast<float *>(&data[offset + z_offset]));
    cyber_point->set_intensity(
        *reinterpret_cast<uint8_t *>(&data[offset + intensity_offset]));
    cyber_point->set_timestamp(static_cast<std::uint64_t>(
        *reinterpret_cast<double *>(&data[offset + stamp_offset]) * 1e9));
  }

  return 1;
}
int main(int argc, char **argv) {
  if (argc != 3) {
    PrintUsage();
    return -1;
  }

  const std::string rosbag_file_name = argv[1];
  const std::string record_file_name = argv[2];
  rosbag::Bag bag;
  try {
    bag.open(rosbag_file_name);
  } catch (...) {
    std::cerr << "Error: the input file is not a ros bag file." << std::endl;
    return -1;
  }
  auto channel_info = ChannelInfo::Instance();

  auto record_writer = std::make_shared<apollo::cyber::record::RecordWriter>();
  record_writer->SetSizeOfFileSegmentation(0);
  record_writer->SetIntervalOfFileSegmentation(0);
  if (!record_writer->Open(record_file_name)) {
    std::cerr << "Error: open file[" << record_file_name << "] failed.";
  }

  ros::Time::init();
  ros::Time start_time = ros::Time::now();

  rosbag::View view(bag,
                    rosbag::TopicQuery(channel_info->GetSupportChannels()));

  std::vector<std::string> channel_write_flag;
  for (const rosbag::MessageInstance m : view) {
    const std::string msg_type = m.getDataType();
    const std::string channel_name = m.getTopic();

    auto desc = channel_info->GetProtoDesc(channel_name);
    auto record_message_type = channel_info->GetMessageType(channel_name);
    if (desc == "" || record_message_type == "") {
      AWARN << "can not find desc or message type for channel: " << channel_name
            << "; desc:" << desc << ";type:" << record_message_type;
    }

    apollo::cyber::proto::Channel channel;
    channel.set_name(channel_name);
    channel.set_message_type(record_message_type);
    channel.set_proto_desc(desc);
    if (std::find(channel_write_flag.begin(), channel_write_flag.end(),
                  channel_name) == channel_write_flag.end() &&
        !record_writer->WriteChannel(channel_name, record_message_type, desc)) {
      AERROR << "write channel info failed";
    } else {
      channel_write_flag.push_back(channel_name);
    }
  }

  for (const rosbag::MessageInstance m : view) {
    const std::string msg_type = m.getDataType();
    const std::string channel_name = m.getTopic();
    uint64_t nsec = m.getTime().toNSec();

    //    auto msg_size = m.size();
    //
    //    std::vector<uint8_t> buffer;
    //    buffer.resize(msg_size);
    //    ros::serialization::IStream stream(buffer.data(), buffer.size());
    //    m.write(stream);
    //
    //    std::string str_msg;
    //    str_msg.reserve(msg_size);
    //    for (size_t i = 0; i < msg_size; ++i) {
    //      str_msg.push_back(buffer[i]);
    //    }
    // FIXME: find a way get all serialized str;
    std::string serialized_str;
    if (channel_name == "/apollo/perception/obstacles") {
      auto pb_msg = m.instantiate<apollo::perception::PerceptionObstacles>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/planning") {
      auto pb_msg = m.instantiate<apollo::planning::ADCTrajectory>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/prediction") {
      auto pb_msg = m.instantiate<apollo::prediction::PredictionObstacles>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/canbus/chassis") {
      auto pb_msg = m.instantiate<apollo::canbus::Chassis>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/control") {
      auto pb_msg = m.instantiate<apollo::control::ControlCommand>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/guardian") {
      auto pb_msg = m.instantiate<apollo::guardian::GuardianCommand>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/localization/pose") {
      auto pb_msg = m.instantiate<apollo::localization::LocalizationEstimate>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/perception/traffic_light") {
      auto pb_msg = m.instantiate<apollo::perception::TrafficLightDetection>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/drive_event") {
      auto pb_msg = m.instantiate<apollo::common::DriveEvent>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/gnss/odometry") {
      auto pb_msg = m.instantiate<apollo::localization::Gps>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/monitor/static_info") {
      auto pb_msg = m.instantiate<apollo::data::StaticInfo>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/monitor") {
      auto pb_msg = m.instantiate<apollo::common::monitor::MonitorMessage>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/canbus/chassis_detail") {
      auto pb_msg = m.instantiate<apollo::canbus::ChassisDetail>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/control/pad") {
      auto pb_msg = m.instantiate<apollo::control::PadMessage>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/navigation") {
      auto pb_msg = m.instantiate<apollo::relative_map::NavigationInfo>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/routing_request") {
      auto pb_msg = m.instantiate<apollo::routing::RoutingRequest>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/routing_response") {
      auto pb_msg = m.instantiate<apollo::routing::RoutingResponse>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/tf" || channel_name == "/tf_static") {
      auto rawdata = m.instantiate<tf2_msgs::TFMessage>();
      auto proto = std::make_shared<apollo::transform::TransformStampeds>();
      apollo::transform::TransformStamped *cyber_tf;
      for (size_t i = 0; i < rawdata->transforms.size(); ++i) {
        cyber_tf = proto->add_transforms();

        cyber_tf->mutable_header()->set_timestamp_sec(
            rawdata->transforms[i].header.stamp.toSec());
        cyber_tf->mutable_header()->set_frame_id(
            rawdata->transforms[i].header.frame_id);
        cyber_tf->mutable_header()->set_sequence_num(
            rawdata->transforms[i].header.seq);

        cyber_tf->set_child_frame_id(rawdata->transforms[i].child_frame_id);

        cyber_tf->mutable_transform()->mutable_translation()->set_x(
            rawdata->transforms[i].transform.translation.x);
        cyber_tf->mutable_transform()->mutable_translation()->set_y(
            rawdata->transforms[i].transform.translation.y);
        cyber_tf->mutable_transform()->mutable_translation()->set_z(
            rawdata->transforms[i].transform.translation.z);
        cyber_tf->mutable_transform()->mutable_rotation()->set_qx(
            rawdata->transforms[i].transform.rotation.x);
        cyber_tf->mutable_transform()->mutable_rotation()->set_qy(
            rawdata->transforms[i].transform.rotation.y);
        cyber_tf->mutable_transform()->mutable_rotation()->set_qz(
            rawdata->transforms[i].transform.rotation.z);
        cyber_tf->mutable_transform()->mutable_rotation()->set_qw(
            rawdata->transforms[i].transform.rotation.w);
      }
      proto->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/conti_radar") {
      auto pb_msg = m.instantiate<apollo::drivers::ContiRadar>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/delphi_esr") {
      auto pb_msg = m.instantiate<apollo::drivers::DelphiESR>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/gnss/best_pose") {
      auto pb_msg = m.instantiate<apollo::drivers::gnss::GnssBestPose>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/gnss/imu") {
      auto pb_msg = m.instantiate<apollo::drivers::gnss::Imu>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/gnss/ins_stat") {
      auto pb_msg = m.instantiate<apollo::drivers::gnss::InsStat>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/gnss/rtk_eph") {
      auto pb_msg = m.instantiate<apollo::drivers::gnss::GnssEphemeris>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/gnss/rtk_obs") {
      auto pb_msg = m.instantiate<apollo::drivers::gnss::EpochObservation>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name ==
               "/apollo/sensor/velodyne64/compensator/PointCloud2") {
      auto ros_msg = m.instantiate<sensor_msgs::PointCloud2>();
      auto pb_msg = std::make_shared<apollo::drivers::PointCloud>();
      convert_PointCloud(pb_msg, ros_msg);
      pb_msg->SerializeToString(&serialized_str);
    } else {
      AWARN << "not support channel:" << channel_name;
      continue;
    }

    auto raw_msg =
        std::make_shared<apollo::cyber::message::RawMessage>(serialized_str);
    if (!record_writer->WriteMessage(channel_name, raw_msg, nsec)) {
      AERROR << "write single msg fail";
    }
  }

  record_writer->Close();
  record_writer = nullptr;
  std::cout << "Info of record file" << std::endl;
  std::string command_line = "cyber_recorder info " + record_file_name;
  int res = system(command_line.c_str());

  std::cout << "Convertion finished! Took " << ros::Time::now() - start_time
            << " seconds in total." << std::endl;
  return res;
}
