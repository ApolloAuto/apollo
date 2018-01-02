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

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/parser.h"
#include "rtcm_parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

class RtcmParserNodelet : public nodelet::Nodelet {
 public:
  RtcmParserNodelet() {}
  ~RtcmParserNodelet() {}

 private:
  virtual void onInit();

  std::unique_ptr<RtcmParser> _rtcm_parser;
};

void RtcmParserNodelet::onInit() {
  ros::NodeHandle& nh = getPrivateNodeHandle();
  std::string rtcm_data_topic;
  std::string eph_topic;
  std::string observation_topic;

  nh.param("rtcm_data_topic", rtcm_data_topic,
           std::string("/apollo/sensor/gnss/rtcm_data"));
  nh.param("eph", eph_topic, std::string("/apollo/sensor/gnss/rtk_eph"));
  nh.param("observation", observation_topic,
           std::string("/apollo/sensor/gnss/rtk_obs"));

  _rtcm_parser.reset(
      new RtcmParser(nh, rtcm_data_topic, eph_topic, observation_topic));
  if (!_rtcm_parser->init()) {
    ROS_ERROR("Init rtcm parser nodelet failed.");
    return;
  }
  ROS_INFO("Init rtcm parser nodelet success.");
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

// Register this plugin with pluginlib.  Names must match nodelet_gnss.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(gnss_driver, RtcmParserNodelet,
                        apollo::drivers::gnss::RtcmParserNodelet,
                        nodelet::Nodelet);
