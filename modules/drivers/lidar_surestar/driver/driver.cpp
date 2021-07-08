/******************************************************************************
 * copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar_surestar/driver/driver.h"

#include <math.h>
#include <time.h>

#include <atomic>
#include <cmath>
#include <memory>
#include <string>

namespace apollo {
namespace drivers {
namespace surestar {

SurestarDriver::SurestarDriver()
    : _basetime(0), _last_gps_time(0), _last_count(0) {}

void SurestarDriver::set_base_time_from_nmea_time(const NMEATimePtr& nmea_time,
                                                  uint64_t* basetime,
                                                  bool gps_time) {
  tm time;
  memset(&time, 0, sizeof(time));

  time.tm_year = nmea_time->year + (2000 - 1900);
  time.tm_mon = nmea_time->mon - 1;
  time.tm_mday = nmea_time->day;
  time.tm_hour = nmea_time->hour;  //+ _config.time_zone();
  time.tm_min = 0;
  time.tm_sec = 0;

  // set last gps time using gps socket packet
  _last_gps_time = (nmea_time->min * 60 + nmea_time->sec) * 1e6;

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

bool SurestarDriver::set_base_time() {
  NMEATimePtr nmea_time(new NMEATime);
  if (_config.use_gps_time()) {
    while (true) {
      int rc = _input->get_positioning_data_packtet(nmea_time);
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
  set_base_time_from_nmea_time(nmea_time, &_basetime);
  _input->init(_config.firing_data_port());
  return true;
}

int SurestarDriver::poll_standard(
    const std::shared_ptr<apollo::drivers::Surestar::SurestarScan>& scan) {
  // Since the surestar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  for (int32_t i = 0; i < _config.npackets(); ++i) {
    while (true) {
      apollo::drivers::Surestar::SurestarPacket* packet;
      // keep reading until full packet received
      packet = scan->add_firing_pkts();
      int rc = _input->get_firing_data_packet(packet);
      if (rc == 0) {
        break;
      }

      if (rc < 0) {
        return rc;
      }
    }
  }

  return 0;
}

int SurestarDriver::poll_sync_count(
    const std::shared_ptr<apollo::drivers::Surestar::SurestarScan>& scan,
    bool main_frame) {
  static std::atomic_ullong sync_counter(0);
  if (main_frame) {
    for (int32_t i = 0; i < _config.npackets(); ++i) {
      while (true) {
        apollo::drivers::Surestar::SurestarPacket* packet;
        // keep reading until full packet received
        packet = scan->add_firing_pkts();
        int rc = _input->get_firing_data_packet(packet);
        if (rc == 0) {
          break;
        }
        if (rc < 0) {
          return rc;
        }
      }
    }
    sync_counter++;
  } else {
    while (scan->firing_pkts_size() < _config.npackets()) {
      while (true) {
        apollo::drivers::Surestar::SurestarPacket* packet;
        // keep reading until full packet received
        packet = scan->add_firing_pkts();
        int rc = _input->get_firing_data_packet(packet);
        if (rc == 0) {
          break;
        }
        if (rc < 0) {
          return rc;
        }
      }
    }
    _last_count = sync_counter;
  }
  return 0;
}

void SurestarDriver::update_gps_top_hour(uint32_t current_time) {
  if (!flags) {
    AINFO << "init current_time:" << current_time
          << ", last_gps_time:" << _last_gps_time;
    flags = true;
  }
  if (_last_gps_time == 0) {
    _last_gps_time = current_time;
    return;
  }
  if (_last_gps_time > current_time) {
    if (std::fabs(_last_gps_time - current_time) > 3599000000) {
      _basetime += 3600;
      AINFO << "update_gps_top_hour. current:" << current_time
            << ", last time:" << _last_gps_time;
    } else {
      // ROS_WARN_STREAM("[driver.cpp] Currrnt stamp:" << std::fixed <<
      // current_time
      //         << " less than previous statmp:" << _last_gps_time
      //         << ". GPS time stamp maybe incorrect!");
    }
  }
  _last_gps_time = current_time;
}

SurestarDriver* SurestarDriverFactory::create_driver(
    const apollo::drivers::surestar::SurestarConfig& surestar_config) {
  if (surestar_config.model() == apollo::drivers::Surestar::Model::RFANS16) {
    return new Surestar16Driver(surestar_config);
  } else {
    AERROR << "Invalid model, must be RFANS16";
    return nullptr;
  }
}
}  // namespace surestar
}  // namespace drivers
}  // namespace apollo
