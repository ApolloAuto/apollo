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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/message/message_traits.h"
#include "cyber/message/protobuf_factory.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/common/monitor_log/proto/monitor_log.pb.h"
#include "modules/common/proto/drive_event.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/data/proto/static_info.pb.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace data {

class ChannelInfo {
 public:
  using StrStrMap = std::unordered_map<std::string, std::string>;

  virtual ~ChannelInfo();

  static const std::shared_ptr<ChannelInfo>& Instance() {
    static auto instance = std::shared_ptr<ChannelInfo>(new ChannelInfo());
    return instance;
  }

  const std::string GetMessageType(const std::string& channel_name);
  const std::string GetProtoDesc(const std::string& channel_name);
  const std::vector<std::string>& GetSupportChannels();

 private:
  ChannelInfo();
  ChannelInfo(const ChannelInfo&) = delete;
  ChannelInfo& operator=(const ChannelInfo&) = delete;

  template <typename M>
  void InitChannelInfo(const std::string& channel_name,
                       const std::string& msg_type) {
    channel_msg_type_[channel_name] = msg_type;
    support_channels_.push_back(channel_name);
    std::string proto_desc("");
    M m;
    apollo::cyber::message::ProtobufFactory::Instance()->GetDescriptorString(
        msg_type, &proto_desc);
    channel_proto_desc_[channel_name] = proto_desc;
  }

  void InitChannels();

  StrStrMap channel_msg_type_;
  StrStrMap channel_proto_desc_;
  std::vector<std::string> support_channels_;
};

}  // namespace data
}  // namespace apollo
