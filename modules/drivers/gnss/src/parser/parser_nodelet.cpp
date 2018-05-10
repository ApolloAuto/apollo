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

#include "data_parser.h"
#include "gnss/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

class ParserNodelet : public nodelet::Nodelet {
 public:
  ParserNodelet() {}
  ~ParserNodelet() {}

 private:
  virtual void onInit();

  std::unique_ptr<DataParser> _data_parser;
};

void ParserNodelet::onInit() {
  ros::NodeHandle& nh = getPrivateNodeHandle();
  std::string gnss_conf;
  std::string raw_data_topic;
  std::string imu_topic;
  std::string ins_stat_topic;
  std::string corr_imu_topic;
  std::string odometry_topic;
  std::string gnss_status_topic;
  std::string ins_status_topic;
  std::string bestpos_topic;
  std::string eph_topic;
  std::string observation_topic;

  nh.param("gnss_conf", gnss_conf, std::string("./conf/gnss_conf.txt"));
  nh.param("raw_data_topic", raw_data_topic,
           std::string("/apollo/sensor/gnss/raw_data"));
  nh.param("imu_topic", imu_topic, std::string("/apollo/sensor/gnss/imu"));
  nh.param("ins_stat_topic", ins_stat_topic,
           std::string("/apollo/sensor/gnss/ins_stat"));
  nh.param("corr_imu_topic", corr_imu_topic,
           std::string("/apollo/sensor/gnss/corrected_imu"));
  nh.param("odometry_topic", odometry_topic,
           std::string("/apollo/sensor/gnss/odometry"));
  nh.param("gnss_status_topic", gnss_status_topic,
           std::string("/apollo/sensor/gnss/gnss_status"));
  nh.param("ins_status_topic", ins_status_topic,
           std::string("/apollo/sensor/gnss/ins_status"));
  nh.param("bestpos", bestpos_topic,
           std::string("/apollo/sensor/gnss/best_pose"));
  nh.param("eph", eph_topic, std::string("/apollo/sensor/gnss/rtk_eph"));
  nh.param("observation", observation_topic,
           std::string("/apollo/sensor/gnss/rtk_obs"));

  _data_parser.reset(new DataParser(
      nh, raw_data_topic, imu_topic, ins_stat_topic, corr_imu_topic,
      odometry_topic, gnss_status_topic, ins_status_topic, bestpos_topic,
      eph_topic, observation_topic));
  if (!_data_parser->init(gnss_conf)) {
    ROS_ERROR("Init parser nodelet failed.");
    ROS_ERROR_STREAM("Init parser nodelet failed.");
    return;
  }
  ROS_INFO("Init parser nodelet success.");
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

// Register this plugin with pluginlib.  Names must match nodelet_gnss.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(gnss_driver, ParserNodelet,
                        apollo::drivers::gnss::ParserNodelet, nodelet::Nodelet);
