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

#ifndef MODULES_TOOLS_ROSBAG_TO_RECORD_CHANNEL_INFO_H_
#define MODULES_TOOLS_ROSBAG_TO_RECORD_CHANNEL_INFO_H_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "cybertron/message/message_traits.h"
#include "cybertron/message/protobuf_factory.h"
#include "cybertron/common/log.h"

#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/common/proto/drive_event.pb.h"

namespace apollo {
namespace tools {

using namespace apollo::cybertron::message;
using apollo::cybertron::message::ProtobufFactory;

class ChannelInfo {
 public:
  using StrStrMap = std::unordered_map<std::string, std::string>;

  virtual ~ChannelInfo();

  static const std::shared_ptr<ChannelInfo>& Instance() {
    static auto instance = std::shared_ptr<ChannelInfo>(new ChannelInfo());
    return instance;
  }

  const std::string& GetMessageType(const std::string& channel_name);
  const std::string& GetProtoDesc(const std::string& channel_name);
  const std::vector<std::string>& GetSupportChannels();

 private:
  ChannelInfo();
  ChannelInfo(const ChannelInfo&) = delete;
  ChannelInfo& operator=(const ChannelInfo&) = delete;
  
  template<typename M>
  void InitChannelInfo(const std::string& channel_name, const std::string& msg_type) {
    channel_msg_type_[channel_name] = msg_type;
    support_channels_.push_back(channel_name);
    std::string proto_desc("");
    M m;
    ProtobufFactory::Instance()->GetDescriptorString(
        msg_type, &proto_desc);
    channel_proto_desc_[channel_name] = proto_desc;
  }

  void InitChannels();

  StrStrMap channel_msg_type_;
  StrStrMap channel_proto_desc_;
  std::vector<std::string> support_channels_;
};

}  // namespace tools
}  // namespace apollo

#endif  // MODULES_TOOLS_ROSBAG_TO_RECORD_CHANNEL_INFO_H_
