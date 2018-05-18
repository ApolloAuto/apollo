/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <std_msgs/String.h>
#include <memory>
#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/drivers/gnss/gnss_gflags.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_parser.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::drivers::gnss::GnssEphemeris;
using ::apollo::drivers::gnss::EpochObservation;
using ::apollo::common::adapter::AdapterManager;

bool RtcmParser::Init() {
  rtcm_parser_.reset(Parser::CreateRtcmV3(true));
  if (!rtcm_parser_) {
    AERROR << "Failed to create rtcm parser.";
    return false;
  }

  inited_flag_ = true;
  return true;
}

void RtcmParser::ParseRtcmData(const std_msgs::String::ConstPtr &msg) {
  if (!inited_flag_) {
    return;
  }

  rtcm_parser_->Update(msg->data);
  Parser::MessageType type;
  MessagePtr msg_ptr;

  while (ros::ok()) {
    type = rtcm_parser_->GetMessage(&msg_ptr);
    if (type == Parser::MessageType::NONE) break;
    DispatchMessage(type, msg_ptr);
  }
}

void RtcmParser::DispatchMessage(Parser::MessageType type, MessagePtr message) {
  std_msgs::String msg_pub;

  switch (type) {
    case Parser::MessageType::EPHEMERIDES:
      PublishEphemeris(message);
      break;

    case Parser::MessageType::OBSERVATION:
      PublishObservation(message);
      break;

    default:
      break;
  }
}

void RtcmParser::PublishEphemeris(const MessagePtr message) {
  GnssEphemeris eph = *As<GnssEphemeris>(message);
  AdapterManager::PublishGnssRtkEph(eph);
}

void RtcmParser::PublishObservation(const MessagePtr message) {
  EpochObservation observation = *As<EpochObservation>(message);
  AdapterManager::PublishGnssRtkObs(observation);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
