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

#include <signal.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "gnss/stream.h"
#include "raw_stream.h"

namespace {

void init_signal(void) {
  signal(SIGPIPE, SIG_IGN);  // ignore SIGPIPE
  signal(SIGSYS, SIG_IGN);   // ignore SIGSYS
}
}

namespace apollo {
namespace drivers {
namespace gnss {

class StreamNodelet : public nodelet::Nodelet {
 public:
  StreamNodelet() {}
  ~StreamNodelet() {}

 private:
  virtual void onInit();
  std::unique_ptr<RawStream> _raw_stream;
};

void StreamNodelet::onInit() {
  ros::NodeHandle& nh = getPrivateNodeHandle();
  std::string gnss_conf;
  std::string raw_data_topic;
  std::string rtcm_data_topic;
  std::string stream_status_topic;

  nh.param("gnss_conf", gnss_conf, std::string("./conf/gnss_conf.txt"));
  nh.param("raw_data_topic", raw_data_topic,
           std::string("/apollo/sensor/gnss/raw_data"));
  nh.param("rtcm_data_topic", rtcm_data_topic,
           std::string("/apollo/sensor/gnss/rtcm_data"));
  nh.param("stream_status_topic", stream_status_topic,
           std::string("/apollo/sensor/gnss/stream_status"));

  ROS_INFO_STREAM("gnss conf: " << gnss_conf);
  ROS_INFO_STREAM("raw data topic: " << raw_data_topic);

  init_signal();
  _raw_stream.reset(new RawStream(nh, getName(), raw_data_topic,
                                  rtcm_data_topic, stream_status_topic));
  if (!_raw_stream->init(gnss_conf)) {
    ROS_ERROR("Init stream nodelet failed.");
    ROS_ERROR_STREAM("Init stream nodelet failed.");
    return;
  }
  ROS_INFO("Init stream nodelet success.");
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

// Register this plugin with pluginlib.  Names must match nodelet_gnss.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(gnss_driver, StreamNodelet,
                        apollo::drivers::gnss::StreamNodelet, nodelet::Nodelet);
