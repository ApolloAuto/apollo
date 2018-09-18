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

#include <cmath>
#include <ctime>
#include <string>

#include "cybertron/cybertron.h"

#include "modules/drivers/velodyne/driver/driver.h"
#include "modules/drivers/velodyne/proto/config.pb.h"
#include "modules/drivers/velodyne/proto/velodyne.pb.h"

namespace apollo {
namespace drivers {
namespace velodyne {

VelodyneDriver::VelodyneDriver() : basetime_(0), last_gps_time_(0) {}

void VelodyneDriver::set_base_time_from_nmea_time(NMEATimePtr nmea_time,
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

bool VelodyneDriver::set_base_time() {
  NMEATimePtr nmea_time(new NMEATime);
  while (true) {
    int rc = input_->get_positioning_data_packet(nmea_time);
    if (rc == 0) {
      break;  // got a full packet
    }
    if (rc < 0) {
      return false;  // end of file reached
    }
  }

  set_base_time_from_nmea_time(nmea_time, &basetime_);
  input_->init(config_.firing_data_port());
  return true;
}

int VelodyneDriver::poll_standard(std::shared_ptr<VelodyneScan> scan) {
  // Since the velodyne delivers data at a very high rate, keep reading and
  // publishing scans as fast as possible.
  // scan->packets.resize(config_.npackets);
  for (int i = 0; i < config_.npackets(); ++i) {
    while (true) {
      // keep reading until full packet received
      VelodynePacket* packet = scan->add_firing_pkts();
      int rc = input_->get_firing_data_packet(packet);

      if (rc == 0) {
        break;  // got a full packet?
      } else if (rc < 0) {
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
      AWARN << "Current stamp:" << std::fixed << current_time
            << " less than previous stamp:" << last_gps_time_
            << ". GPS time stamp maybe incorrect!";
    }
  }
  last_gps_time_ = current_time;
}

VelodyneDriver* VelodyneDriverFactory::create_driver(const Config& config) {
  Config new_config = config;
  if (new_config.prefix_angle() > 35900 || new_config.prefix_angle() < 100) {
    AWARN << "invalid prefix angle, prefix_angle must be between 100 and 35900";
    if (new_config.prefix_angle() > 35900) {
      new_config.set_prefix_angle(35900);
    } else if (new_config.prefix_angle() < 100) {
      new_config.set_prefix_angle(100);
    }
  }
  if (config.model() == HDL64E_S2 || config.model() == HDL64E_S3S ||
      config.model() == HDL64E_S3D) {
    return new Velodyne64Driver(config);
  } else if (config.model() == HDL32E) {
    return new Velodyne32Driver(config);
  } else if (config.model() == VLP16) {
    return new Velodyne16Driver(config);
  } else if (config.model() == VLS128) {
    return nullptr;
  } else {
    AERROR << "invalid model, must be 64E_S2|64E_S3S"
           << "|64E_S3D|VLP16|HDL32E|VLS128";
    return nullptr;
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
