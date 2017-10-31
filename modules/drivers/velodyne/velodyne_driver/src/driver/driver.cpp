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

#include "driver.h"

#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <string>

namespace apollo {
namespace drivers {
namespace velodyne {

VelodyneDriver::VelodyneDriver() : basetime_(0), last_gps_time_(0) {}

void VelodyneDriver::set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
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

bool VelodyneDriver::set_base_time() {
  NMEATimePtr nmea_time(new NMEATime);
  while (true) {
    int rc = input_->get_positioning_data_packtet(nmea_time);
    if (rc == 0) {
      break;  // got a full packet
    }
    if (rc < 0) {
      return false;  // end of file reached
    }
  }

  set_base_time_from_nmea_time(nmea_time, basetime_);
  input_->init(config_.firing_data_port);
  return true;
}

int VelodyneDriver::poll_standard(velodyne_msgs::VelodyneScanUnifiedPtr& scan) {
  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  scan->packets.resize(config_.npackets);
  for (int i = 0; i < config_.npackets; ++i) {
    while (true) {
      // keep reading until full packet received
      int rc = input_->get_firing_data_packet(&scan->packets[i]);

      if (rc == 0) {
        break;  // got a full packet?
      }

      if (rc < 0) {
        return rc;
      }
    }
  }

  return 0;
}

void VelodyneDriver::update_gps_top_hour(uint32_t current_time) {
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

VelodyneDriver* VelodyneDriverFactory::create_driver(
    ros::NodeHandle private_nh) {
  Config config;
  // use private node handle to get parameters
  private_nh.param("frame_id", config.frame_id, std::string("velodyne"));
  private_nh.param("model", config.model, std::string("64E"));
  private_nh.param("topic", config.topic, std::string("velodyne_packets"));
  private_nh.param("firing_data_port", config.firing_data_port,
                   FIRING_DATA_PORT);
  private_nh.param("positioning_data_port", config.positioning_data_port,
                   POSITIONING_DATA_PORT);
  private_nh.param("rpm", config.rpm, 600.0);

  if (config.model == "64E_S2" || config.model == "64E_S3S" ||
      config.model == "64E_S3D_STRONGEST" || config.model == "64E_S3D_LAST" ||
      config.model == "64E_S3D_DUAL") {
    return new Velodyne64Driver(config);
  } else if (config.model == "VLP16") {
    return new Velodyne16Driver(config);
  } else {
    ROS_ERROR_STREAM("invalid model, must be 64E_S2|64E_S3S"
                     << "|64E_S3D_STRONGEST|64E_S3D_LAST|64E_S3D_DUAL|VLP16");
    return nullptr;
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
