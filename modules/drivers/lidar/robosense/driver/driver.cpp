/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/robosense/driver/driver.h"

#include <atomic>
#include <cmath>
#include <ctime>
#include <memory>
#include <string>

namespace apollo {
namespace drivers {
namespace robosense {

RobosenseDriver::RobosenseDriver()
    : basetime_(0), last_gps_time_(0), last_count_(0) {}

void RobosenseDriver::set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
                                                   uint64_t* basetime,
                                                   bool gps_time) {
  tm time;
  memset(&time, 0, sizeof(time));

  time.tm_year = nmea_time->year + (2000 - 1900);
  time.tm_mon = nmea_time->mon - 1;
  time.tm_mday = nmea_time->day;
  time.tm_hour = nmea_time->hour;  //+ config_.time_zone();
  time.tm_min = 0;
  time.tm_sec = 0;

  // set last gps time using gps socket packet
  last_gps_time_ = (nmea_time->min * 60 + nmea_time->sec) * 1e6;

  AINFO << "Set base unix time : " << time.tm_year << "-" << time.tm_mon << "-"
        << time.tm_mday << " " << time.tm_hour << ":" << time.tm_min << ":"
        << time.tm_sec;

  uint64_t unix_base = 0;
  if (!gps_time) {
    AINFO << "local time--------------------------";
    unix_base = static_cast<uint64_t>(mktime(&time));
  } else {
    AINFO << "gps time---------------------------";
    unix_base = static_cast<uint64_t>(timegm(&time));
  }
  *basetime = unix_base;  //* static_cast<uint64_t>(1e6);
}

bool RobosenseDriver::set_base_time() {
  NMEATimePtr nmea_time(new NMEATime);
  if (config_.use_gps_time()) {
    while (true) {
      int rc = input_->get_positioning_data_packtet(nmea_time);
      if (rc == 0) {
        break;  // got a full packet
      }
      if (rc < 0) {
        return false;  // end of file reached
      }
    }
  } else {
    time_t t = time(NULL);
    struct tm ptm;
    struct tm* current_time = localtime_r(&t, &ptm);
    nmea_time->year = current_time->tm_year - 100;
    nmea_time->mon = current_time->tm_mon + 1;
    nmea_time->day = current_time->tm_mday;
    nmea_time->hour = current_time->tm_hour;
    nmea_time->min = current_time->tm_min;
    nmea_time->sec = current_time->tm_sec;
  }
  set_base_time_from_nmea_time(nmea_time, &basetime_);
  input_->init(config_.firing_data_port());
  return true;
}

int RobosenseDriver::poll_standard(
    const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan) {
  // Since the suteng delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  for (int32_t i = 0; i < config_.npackets(); ++i) {
    while (true) {
      apollo::drivers::suteng::SutengPacket* packet;
      // keep reading until full packet received
      packet = scan->add_firing_pkts();
      int rc = input_->get_firing_data_packet(packet, i, start_time_);
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

int RobosenseDriver::poll_sync_count(
    const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan,
    bool main_frame) {
  static std::atomic_ullong sync_counter(0);
  int time_zone = config_.time_zone();
  // apollo::drivers::suteng::SutengPacket* tmp_packet;
  if (main_frame) {
    for (int32_t i = 0; i < config_.npackets(); ++i) {
      while (true) {
        apollo::drivers::suteng::SutengPacket* packet;
        // keep reading until full packet received
        packet = scan->add_firing_pkts();
        int rc = input_->get_firing_data_packet(packet, time_zone, start_time_);
        // tmp_packet = packet;
        if (rc == 0) {
          break;  // got a full packet?
        }
        if (rc < 0) {
          return rc;
        }
      }
    }
    sync_counter++;
  } else {
    int pk_i = 0;
    while (scan->firing_pkts_size() < config_.npackets()) {
      while (true) {
        apollo::drivers::suteng::SutengPacket* packet;
        // keep reading until full packet received
        packet = scan->add_firing_pkts();
        int rc = input_->get_firing_data_packet(packet, time_zone, start_time_);
        pk_i++;
        if (rc == 0) {
          break;
        }
        if (rc < 0) {
          return rc;
        }
      }
    }
    last_count_ = sync_counter;
  }

  return 0;
}
static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
static int last_azimuth = ANGLE_HEAD;
bool RobosenseDriver::cute_angle(
    apollo::drivers::suteng::SutengPacket* packet) {
  int azimuth = 256 * packet->data().c_str()[44] + packet->data().c_str()[45];
  if (azimuth < last_azimuth) {
    last_azimuth -= 36000;
  }
  // Check if currently passing cut angle
  if (last_azimuth != ANGLE_HEAD && last_azimuth < 1 && azimuth >= 1) {
    last_azimuth = azimuth;
    return false;  // Cut angle passed, one full revolution collected
  }
  last_azimuth = azimuth;
  return true;
}
void RobosenseDriver::update_gps_top_hour(uint32_t current_time) {
  if (!flags) {
    AINFO << "init current_time:" << current_time
          << ", last_gps_time:" << last_gps_time_;
    flags = true;
  }
  if (last_gps_time_ == 0) {
    last_gps_time_ = current_time;
    return;
  }
  if (last_gps_time_ > current_time) {
    if (std::fabs(last_gps_time_ - current_time) > 3599000000) {
      basetime_ += 3600;
      AINFO << "update_gps_top_hour. current:" << current_time
            << ", last time:" << last_gps_time_;
    }
  }
  last_gps_time_ = current_time;
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
