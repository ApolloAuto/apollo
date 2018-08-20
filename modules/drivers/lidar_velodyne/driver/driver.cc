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

#include "modules/drivers/lidar_velodyne/driver/driver.h"

#include <cmath>
#include <set>
#include <string>

#include "modules/common/log.h"
#include "modules/drivers/lidar_velodyne/common/velodyne_gflags.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

VelodyneDriver::VelodyneDriver() : basetime_(0), last_gps_time_(0) {}

void VelodyneDriver::set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
                                                  uint64_t* basetime) {
  tm time;
  time.tm_year = nmea_time->year + (2000 - 1900);
  time.tm_mon = nmea_time->mon - 1;
  time.tm_mday = nmea_time->day;
  time.tm_hour = nmea_time->hour;
  time.tm_min = 0;
  time.tm_sec = 0;

  // set last gps time using gps socket packet
  last_gps_time_ = (nmea_time->min * 60 + nmea_time->sec) * 1e6;
  AINFO << "Set base unix time : " << time.tm_year << "-" << time.tm_mon << "-"
        << time.tm_mday << " " << time.tm_hour << ":" << time.tm_min << ":"
        << time.tm_sec;
  uint64_t unix_base = static_cast<uint64_t>(timegm(&time));
  *basetime = unix_base * 1e6;
}

int VelodyneDriver::poll_standard(velodyne_msgs::VelodyneScanUnifiedPtr scan) {
  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  scan->packets.resize(config_.npackets());
  int size = config_.npackets();
  if (FLAGS_pipeline_mode) {
    size = 1;
  }
  for (int i = 0; i < size; ++i) {
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
      AINFO << "Base time plus 3600s. Model: " << config_.model() << std::fixed
            << ". current:" << current_time << ", last time:" << last_gps_time_;
    } else {
      AWARN << "Currrnt stamp:" << std::fixed << current_time
            << " less than previous statmp:" << last_gps_time_
            << ". GPS time stamp maybe incorrect!";
    }
  }
  last_gps_time_ = current_time;
}

VelodyneDriver* VelodyneDriverFactory::create_driver(const VelodyneConf& conf) {
  if (conf.model() == VLP16) {
    return new Velodyne16Driver(conf);
  } else {
    return new Velodyne64Driver(conf);
  }
}

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo
