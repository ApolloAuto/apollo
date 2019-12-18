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

#include "modules/drivers/velodyne/driver/driver.h"

#include <cmath>
#include <ctime>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/drivers/velodyne/proto/config.pb.h"
#include "modules/drivers/velodyne/proto/velodyne.pb.h"

namespace apollo {
namespace drivers {
namespace velodyne {

uint64_t VelodyneDriver::sync_counter = 0;

VelodyneDriver::~VelodyneDriver() {
  if (positioning_thread_.joinable()) {
    positioning_thread_.join();
  }
}

void VelodyneDriver::Init() {
  double frequency = (config_.rpm() / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.set_npackets(static_cast<int>(ceil(packet_rate_ / frequency)));
  AINFO << "publishing " << config_.npackets() << " packets per scan";

  // open Velodyne input device

  input_.reset(new SocketInput());
  positioning_input_.reset(new SocketInput());
  input_->init(config_.firing_data_port());
  positioning_input_->init(config_.positioning_data_port());

  // raw data output topic
  positioning_thread_ =
      std::thread(&VelodyneDriver::PollPositioningPacket, this);
}

void VelodyneDriver::SetBaseTimeFromNmeaTime(NMEATimePtr nmea_time,
                                             uint64_t* basetime) {
  tm time;
  time.tm_year = nmea_time->year + (2000 - 1900);
  time.tm_mon = nmea_time->mon - 1;
  time.tm_mday = nmea_time->day;
  time.tm_hour = nmea_time->hour;
  time.tm_min = 0;
  time.tm_sec = 0;

  // set last gps time using gps socket packet
  last_gps_time_ =
      static_cast<uint32_t>((nmea_time->min * 60 + nmea_time->sec) * 1e6);

  AINFO << "Set base unix time : " << time.tm_year << "-" << time.tm_mon << "-"
        << time.tm_mday << " " << time.tm_hour << ":" << time.tm_min << ":"
        << time.tm_sec;
  uint64_t unix_base = static_cast<uint64_t>(timegm(&time));
  *basetime = unix_base * static_cast<uint64_t>(1e6);
}

bool VelodyneDriver::SetBaseTime() {
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

  SetBaseTimeFromNmeaTime(nmea_time, &basetime_);
  input_->init(config_.firing_data_port());
  return true;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::Poll(const std::shared_ptr<VelodyneScan>& scan) {
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  if (basetime_ == 0) {
    // waiting for positioning data
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    AWARN << "basetime is zero";
    return false;
  }

  int poll_result = PollStandard(scan);

  if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL) {
    return false;  // poll again
  }

  if (scan->firing_pkts().empty()) {
    AINFO << "Get an empty scan from port: " << config_.firing_data_port();
    return false;
  }

  // publish message using time of last packet read
  ADEBUG << "Publishing a full Velodyne scan.";
  scan->mutable_header()->set_timestamp_sec(cyber::Time().Now().ToSecond());
  scan->mutable_header()->set_frame_id(config_.frame_id());
  scan->set_model(config_.model());
  scan->set_mode(config_.mode());
  // we use first packet gps time update gps base hour
  // in cloud nodelet, will update base time packet by packet
  uint32_t current_secs = *(reinterpret_cast<uint32_t*>(
      const_cast<char*>(scan->firing_pkts(0).data().data() + 1200)));

  UpdateGpsTopHour(current_secs);
  scan->set_basetime(basetime_);

  return true;
}

int VelodyneDriver::PollStandard(std::shared_ptr<VelodyneScan> scan) {
  // Since the velodyne delivers data at a very high rate, keep reading and
  // publishing scans as fast as possible.
  while ((config_.use_poll_sync() &&
          ((config_.is_main_frame() &&
            scan->firing_pkts_size() < config_.npackets()) ||
           (!config_.is_main_frame() && sync_counter == last_count_))) ||
         (!config_.use_poll_sync() &&
          scan->firing_pkts_size() < config_.npackets())) {
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
  if (config_.use_poll_sync()) {
    if (config_.is_main_frame()) {
      ++sync_counter;
    } else {
      last_count_ = sync_counter;
    }
  }
  return 0;
}

void VelodyneDriver::PollPositioningPacket(void) {
  while (!cyber::IsShutdown()) {
    NMEATimePtr nmea_time(new NMEATime);
    bool ret = true;
    if (config_.has_use_gps_time() && !config_.use_gps_time()) {
      time_t t = time(NULL);
      struct tm current_time;
      localtime_r(&t, &current_time);
      nmea_time->year = static_cast<uint16_t>(current_time.tm_year - 100);
      nmea_time->mon = static_cast<uint16_t>(current_time.tm_mon + 1);
      nmea_time->day = static_cast<uint16_t>(current_time.tm_mday);
      nmea_time->hour = static_cast<uint16_t>(current_time.tm_hour);
      nmea_time->min = static_cast<uint16_t>(current_time.tm_min);
      nmea_time->sec = static_cast<uint16_t>(current_time.tm_sec);
      AINFO << "Get NMEA Time from local time :"
            << "year:" << nmea_time->year << "mon:" << nmea_time->mon
            << "day:" << nmea_time->day << "hour:" << nmea_time->hour
            << "min:" << nmea_time->min << "sec:" << nmea_time->sec;
    } else {
      while (!cyber::IsShutdown()) {
        int rc = positioning_input_->get_positioning_data_packet(nmea_time);
        if (rc == 0) {
          break;  // got a full packet
        }
        if (rc < 0) {
          ret = false;  // end of file reached
        }
      }
    }

    if (basetime_ == 0 && ret) {
      SetBaseTimeFromNmeaTime(nmea_time, &basetime_);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void VelodyneDriver::UpdateGpsTopHour(uint32_t current_time) {
  if (last_gps_time_ == 0) {
    last_gps_time_ = current_time;
    return;
  }
  if (last_gps_time_ > current_time) {
    if (std::abs(last_gps_time_ - current_time) > 3599000000) {
      basetime_ += static_cast<uint64_t>(3600 * 1e6);
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

VelodyneDriver* VelodyneDriverFactory::CreateDriver(const Config& config) {
  auto new_config = config;
  if (new_config.prefix_angle() > 35900 || new_config.prefix_angle() < 100) {
    AWARN << "invalid prefix angle, prefix_angle must be between 100 and 35900";
    if (new_config.prefix_angle() > 35900) {
      new_config.set_prefix_angle(35900);
    } else if (new_config.prefix_angle() < 100) {
      new_config.set_prefix_angle(100);
    }
  }
  VelodyneDriver* driver = nullptr;
  switch (config.model()) {
    case HDL64E_S2: {
      driver = new Velodyne64Driver(config);
      driver->SetPacketRate(PACKET_RATE_HDL64E_S2);
      break;
    }
    case HDL64E_S3S: {
      driver = new Velodyne64Driver(config);
      driver->SetPacketRate(PACKET_RATE_HDL64E_S3S);
      break;
    }
    case HDL64E_S3D: {
      driver = new Velodyne64Driver(config);
      driver->SetPacketRate(PACKET_RATE_HDL64E_S3D);
      break;
    }
    case HDL32E: {
      driver = new VelodyneDriver(config);
      driver->SetPacketRate(PACKET_RATE_HDL32E);
      break;
    }
    case VLP32C: {
      driver = new VelodyneDriver(config);
      driver->SetPacketRate(PACKET_RATE_VLP32C);
      break;
    }
    case VLP16: {
      driver = new VelodyneDriver(config);
      driver->SetPacketRate(PACKET_RATE_VLP16);
      break;
    }
    case VLS128: {
      driver = new VelodyneDriver(config);
      driver->SetPacketRate(PACKET_RATE_VLS128);
      break;
    }
    default:
      AERROR << "invalid model, must be 64E_S2|64E_S3S"
             << "|64E_S3D|VLP16|HDL32E|VLS128";
      break;
  }
  return driver;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
