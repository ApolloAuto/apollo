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

#ifndef MODULES_DRIVERS_GNSS_DATA_PARSER_H_
#define MODULES_DRIVERS_GNSS_DATA_PARSER_H_

#include <memory>

#include <proj_api.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/parser.h"
#include "proto/config.pb.h"

#include "proto/gnss.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"

#include "proto/gnss_status.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

class DataParser {
 public:
  DataParser(ros::NodeHandle &nh, const std::string &raw_data_topic,
             const std::string &imu_topic, const std::string &gpgga_topic,
             const std::string &corr_imu_topic,
             const std::string &odometry_topic,
             const std::string &gnss_status_topic,
             const std::string &ins_status_topic,
             const std::string &bestpos_topic, const std::string &eph_topic,
             const std::string &observation_topic);
  ~DataParser() {}
  bool init(const std::string &cfg_file);

 private:
  void raw_data_callback(const std_msgs::String::ConstPtr &msg);
  void dispatch_message(Parser::MessageType type, MessagePtr message);
  void publish_ins_stat(const MessagePtr message);
  void publish_odometry_message(const MessagePtr message);
  void publish_corrimu_message(const MessagePtr message);
  void publish_imu_message(const MessagePtr message);
  void publish_bestpos_message(const MessagePtr message);
  void publish_ephemeris(const MessagePtr message);
  void publish_observation(const MessagePtr message);
  void check_ins_status(drivers::gnss::Ins *ins);
  void check_gnss_status(drivers::gnss::Gnss *gnss);

  bool _inited_flag = false;
  std::unique_ptr<Parser> _data_parser;

  const ros::Subscriber _raw_data_sub;
  const ros::Publisher _ins_stat_publisher;
  const ros::Publisher _raw_imu_publisher;
  const ros::Publisher _imu_publisher;
  const ros::Publisher _nav_odometry_publisher;
  const ros::Publisher _gnss_status_publisher;
  const ros::Publisher _ins_status_publisher;
  const ros::Publisher _bestpos_publisher;
  const ros::Publisher _ephemeris_publisher;
  const ros::Publisher _observation_publisher;

  boost::shared_ptr<apollo::drivers::gnss_status::GnssStatus> _gnss_status;
  boost::shared_ptr<apollo::drivers::gnss_status::InsStatus> _ins_status;
  uint32_t _ins_status_record = static_cast<uint32_t>(0);
  projPJ _wgs84pj_source;
  projPJ _utm_target;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_DATA_PARSER_H_
