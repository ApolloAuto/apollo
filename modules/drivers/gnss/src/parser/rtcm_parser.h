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

#ifndef MODULES_DRIVERS_GNSS_RTCM_PARSER_H_
#define MODULES_DRIVERS_GNSS_RTCM_PARSER_H_

#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

class RtcmParser {
 public:
  RtcmParser(ros::NodeHandle &nh, const std::string &rtcm_data_topic,
             const std::string &eph_topic,
             const std::string &observation_topic);
  ~RtcmParser() {}
  bool init();

 private:
  void rtcm_data_callback(const std_msgs::String::ConstPtr &msg);
  void dispatch_message(Parser::MessageType type, MessagePtr message);
  void publish_ephemeris(const MessagePtr message);
  void publish_observation(const MessagePtr message);

  bool _inited_flag = false;
  std::unique_ptr<Parser> _rtcm_parser;

  const ros::Subscriber _rtcm_data_sub;
  const ros::Publisher _ephemeris_publisher;
  const ros::Publisher _observation_publisher;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_RTCM_PARSER_H_
