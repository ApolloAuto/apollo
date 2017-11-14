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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>

#include "gnss/parser.h"
#include "rtcm_parser.h"
#include "proto/gnss_raw_observation.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

RtcmParser::RtcmParser(ros::NodeHandle &nh,
                       const std::string &rtcm_data_topic,
                       const std::string &eph_topic,
                       const std::string &observation_topic)
    : _rtcm_data_sub(nh.subscribe(rtcm_data_topic, 16,
                                  &RtcmParser::rtcm_data_callback, this)),
      _ephemeris_publisher(
       nh.advertise<GnssEphemeris>(eph_topic, 16)),
      _observation_publisher(
       nh.advertise<EpochObservation>(observation_topic, 16)) {}

bool RtcmParser::init() {
  _rtcm_parser.reset(Parser::create_rtcm_v3());
  if (!_rtcm_parser) {
    ROS_ERROR("Failed to create rtcm parser.");
    return false;
  }

  _inited_flag = true;
  return true;
}

void RtcmParser::rtcm_data_callback(const std_msgs::String::ConstPtr& msg) {
  if (!_inited_flag) {
    return;
  }

  _rtcm_parser->update(msg->data);
  Parser::MessageType type;
  MessagePtr msg_ptr;

  for (;;) {
    type = _rtcm_parser->get_message(msg_ptr);
    if (type == Parser::MessageType::NONE) break;
    dispatch_message(type, msg_ptr);
  }
}

void RtcmParser::dispatch_message(Parser::MessageType type,
                                  MessagePtr message) {
  std_msgs::String msg_pub;

  switch (type) {
    case Parser::MessageType::EPHEMERIDES:
      publish_ephemeris(message);
      break;

    case Parser::MessageType::OBSERVATION:
      publish_observation(message);
      break;

    default:
      break;
  }
}

void RtcmParser::publish_ephemeris(const MessagePtr message) {
  boost::shared_ptr<::apollo::drivers::gnss::GnssEphemeris> eph(
    new apollo::drivers::gnss::GnssEphemeris(
          *As<::apollo::drivers::gnss::GnssEphemeris>(message)));

  _ephemeris_publisher.publish(eph);
}

void RtcmParser::publish_observation(const MessagePtr message) {
  boost::shared_ptr<::apollo::drivers::gnss::EpochObservation> observation(
    new ::apollo::drivers::gnss::EpochObservation(
          *As<::apollo::drivers::gnss::EpochObservation>(message)));

  _observation_publisher.publish(observation);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
