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

#include "cybertron/proto/record.pb.h"

#include "modules/tools/rosbag_to_record/rosbag_to_record.h"
#include "modules/tools/rosbag_to_record/channel_info.h"

#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/localization/proto/localization.pb.h"

using apollo::cybertron::proto::SingleMessage;
using apollo::tools::ChannelInfo;

void PrintUsage() {
  std::cout << "Usage:\n"
            << "  rosbag_to_record myfile.bag" << std::endl;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    PrintUsage();
    return -1;
  }

  const std::string rosbag_file_name = argv[1];
  rosbag::Bag bag;
  try {
    bag.open(rosbag_file_name);
  } catch (...) {
    std::cerr << "Error: the input file is not a ros bag file." << std::endl;
    return -1;
  }

  auto channel_info = ChannelInfo::Instance();
  std::cout << "Info of ros bag file" << std::endl;
  std::string command_line = "rosbag info " + rosbag_file_name;
  system(command_line.c_str());

  const std::string record_file_name =
      rosbag_file_name.substr(0, rosbag_file_name.size() - 3) + "record";

  auto record_writer =
      std::make_shared<apollo::cybertron::record::RecordWriter>();
  if (!record_writer->Open(record_file_name)) {
    std::cerr << "Error: open file[" << record_file_name << "] failed.";
  }

  ros::Time::init();
  ros::Time start_time = ros::Time::now();

  rosbag::View view(bag, rosbag::TopicQuery(channel_info->GetSupportChannels()));
  
  for (auto channel_name : channel_info->GetSupportChannels()) {
    auto desc = channel_info->GetProtoDesc(channel_name);
    auto record_message_type = channel_info->GetMessageType(channel_name);
    if (desc == "" || record_message_type == "") {
      AWARN << "can not find desc or message type";
    }
    if (!record_writer->WriteChannel(channel_name, record_message_type, desc)) {
      AERROR << "write channel info failed";
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
    //TODO: find a way get all serialized str;
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
    } else if (channel_name == "/apollo/sensor/gnss/corrected_imu") {
      auto pb_msg = m.instantiate<apollo::localization::CorrectedImu>();
      pb_msg->SerializeToString(&serialized_str);
    } else if (channel_name == "/apollo/sensor/gnss/odometry") {
      auto pb_msg = m.instantiate<apollo::localization::Gps>();
      pb_msg->SerializeToString(&serialized_str);
    } else {
      AWARN << "not support channel:" << channel_name;
      continue;
    }
 
//    auto desc = channel_info->GetProtoDesc(channel_name);
//    auto record_message_type = channel_info->GetMessageType(channel_name);
//    if (desc == "" || record_message_type == "") {
//      AWARN << "can not find desc or message type";
//    }
//    if (!record_writer->WriteChannel(channel_name, record_message_type, desc)) {
//      AERROR << "write channel info failed";
//    }
    SingleMessage single_msg;
    single_msg.set_channel_name(channel_name);
    
    single_msg.set_content(serialized_str);
    single_msg.set_time(nsec);
    if (!record_writer->WriteMessage(single_msg)) {
      AERROR << "write single msg fail";
    }
  }

  record_writer->Close();
  record_writer = nullptr;
  std::cout << "Info of record file" << std::endl;
  command_line = "cyber_recorder info -f " + record_file_name;
  system(command_line.c_str());

  std::cout << "Convertion finished! Took " << ros::Time::now() - start_time
            << " seconds in total." << std::endl;
  return 0;
}
