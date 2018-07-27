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

#include "driver.h"

#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <string>

namespace apollo {
namespace drivers {
namespace rslidar {

RslidarDriver::RslidarDriver() : basetime_(0), last_gps_time_(0) {}

void RslidarDriver::set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
                                                  uint64_t& basetime) {
  tm time;
  time.tm_year = nmea_time->year + (2000 - 1900);
  time.tm_mon = nmea_time->mon - 1;
  time.tm_mday = nmea_time->day;
  time.tm_hour = nmea_time->hour;
  time.tm_min = 0;
  time.tm_sec = 0;

  // set last gps time using gps socket packet
  last_gps_time_ = (nmea_time->min * 60 + nmea_time->sec) * 1e6;

  ROS_INFO("Set base unix time : %d-%d-%d %d:%d:%d", time.tm_year, time.tm_mon,
           time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
  uint64_t unix_base = static_cast<uint64_t>(timegm(&time));
  basetime = unix_base * 1e6;
}

bool RslidarDriver::set_base_time() {
  NMEATimePtr nmea_time(new NMEATime);
  set_base_time_from_nmea_time(nmea_time, basetime_);
  input_->init(config_.msop_data_port);
  return true;
}

int RslidarDriver::poll_standard(rslidar_msgs::rslidarScanPtr& scan) {
  scan->packets.resize(config_.npackets);
  for (int i = 0; i < config_.npackets; ++i) {
    while (true) {
      // keep reading until full packet received
      int rc = input_->get_msop_data_packet(&scan->packets[i]);

      if (rc == 0) {
        break;  // got a full packet?
      }else if (rc < 0) {
        return rc;
      }
    }
  }

  return 0;
}

void RslidarDriver::update_gps_top_hour(uint32_t current_time) {
  if (last_gps_time_ == 0) {
    last_gps_time_ = current_time;
    return;
  }
  if (last_gps_time_ > current_time) {
    if (std::abs(last_gps_time_ - current_time) > 3599000000) {
      basetime_ += 3600 * 1e6;
      ROS_INFO_STREAM("Base time plus 3600s. Model: "
                      << config_.model << std::fixed << ". current:"
                      << current_time << ", last time:" << last_gps_time_);
    } else {
      ROS_WARN_STREAM("Currrnt stamp:" << std::fixed << current_time
                                       << " less than previous statmp:"
                                       << last_gps_time_
                                       << ". GPS time stamp maybe incorrect!");
    }
  }
  last_gps_time_ = current_time;
}

RslidarDriver* RslidarDriverFactory::create_driver(
    ros::NodeHandle private_nh) {
  Config config;
  // use private node handle to get parameters
  private_nh.param("frame_id", config.frame_id, std::string("rslidar"));
  private_nh.param("model", config.model, std::string("RS16"));

  private_nh.param("topic", config.topic, std::string("rslidar_packets"));
  private_nh.param("msop_data_port", config.msop_data_port,
                   MSOP_DATA_PORT);
  private_nh.param("difop_data_port", config.difop_data_port,
                   DIFOP_DATA_PORT);
  
  private_nh.param("rpm", config.rpm, 600.0);

  double packet_rate;
  if (config.model == "RS16") {
  	 packet_rate = 840;
     return new Rslidar16Driver(config);
  } else if (config.model == "RS32") {
  	 packet_rate = 1690;
     return new Rslidar32Driver(config);
  } else {
       ROS_ERROR_STREAM("unknown LIDAR model: " << config.model);
       packet_rate = 2600.0;
	   return nullptr;
  }
}

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo
