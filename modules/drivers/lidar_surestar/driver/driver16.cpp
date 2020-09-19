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

#include <time.h>
#include <unistd.h>

#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "modules/drivers/lidar_surestar/driver/driver.h"

namespace apollo {
namespace drivers {
namespace surestar {

Surestar16Driver::Surestar16Driver(
    const apollo::drivers::surestar::SurestarConfig& surestar_config)
    : SurestarDriver() {
  _config = surestar_config;
}

Surestar16Driver::~Surestar16Driver() {
  running_.store(false);
  if (positioning_thread_.joinable()) {
    positioning_thread_.join();
  }
}

void Surestar16Driver::init() {
  running_.store(true);
  double packet_rate =
      781.25;  // 每秒packet的数目  beike v6k-781.25  v6c-833.33
  double frequency = (_config.rpm() /
                      60.0);  // 每秒frame的数目，即每秒的圈数  expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  _config.set_npackets(
      ceil(packet_rate / frequency));  // 每frame，即转一圈packet的数目

  if (_config.has_pcap_file()) {
    AINFO << "_config.pcap_file(): " << _config.pcap_file();
    _input.reset(
        new surestar::PcapInput(packet_rate, _config.pcap_file(), false));
    _input->init();

    _positioning_input.reset(
        new surestar::PcapInput(packet_rate, _config.pcap_file(), false));
    _positioning_input->init();
    AINFO << "driver16-inited----";
  } else {
    _input.reset(new surestar::SocketInput());
    _input->init(_config.firing_data_port());

    _positioning_input.reset(new surestar::SocketInput());
    _positioning_input->init(_config.positioning_data_port());
  }
  positioning_thread_ =
      std::thread(&Surestar16Driver::poll_positioning_packet, this);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool Surestar16Driver::poll(
    const std::shared_ptr<apollo::drivers::Surestar::SurestarScan>& scan) {
  if (_basetime == 0) {
    AINFO << "poll function  _basetime=0!";
    usleep(100);
    return false;
  }
  int poll_result = SOCKET_TIMEOUT;
  // Synchronizing all laser radars
  if (_config.main_frame()) {
    poll_result = poll_sync_count(scan, true);
  } else {
    poll_result = poll_sync_count(scan, false);
  }
  if (poll_result == PCAP_FILE_END) {
    return false;  // read the end of pcap file, return false stop poll;
  }
  if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL) {
    return false;  // poll again
  }
  if (scan->firing_pkts_size() == 0) {
    AINFO << "Get a empty scan from port: " << _config.firing_data_port();
    return false;
  }
  scan->set_model(_config.model());
  scan->set_mode(_config.mode());
  scan->mutable_header()->set_frame_id(_config.frame_id());
  scan->mutable_header()->set_timestamp_sec(
      apollo::cyber::Time().Now().ToSecond());
  uint32_t current_secs = *(reinterpret_cast<uint32_t*>(
      const_cast<char*>(scan->firing_pkts(0).data().data() + 1200)));
  update_gps_top_hour(current_secs);
  scan->set_basetime(_basetime);

  AINFO << "time: " << apollo::cyber::Time().Now().ToNanosecond();
  return true;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
void Surestar16Driver::poll_positioning_packet(void) {
  while (!apollo::cyber::IsShutdown() && running_.load()) {
    NMEATimePtr nmea_time(new NMEATime);
    bool ret = true;
    if (!_config.use_gps_time()) {
      time_t t = time(NULL);
      struct tm ptm;
      struct tm* current_time = localtime_r(&t, &ptm);
      nmea_time->year = current_time->tm_year - 100;
      nmea_time->mon = current_time->tm_mon + 1;
      nmea_time->day = current_time->tm_mday;
      nmea_time->hour = current_time->tm_hour;
      nmea_time->min = current_time->tm_min;
      nmea_time->sec = current_time->tm_sec;
      AINFO << "frame_id:" << _config.frame_id() << "-F(local-time):"
            << "year:" << nmea_time->year << "mon:" << nmea_time->mon
            << "day:" << nmea_time->day << "hour:" << nmea_time->hour
            << "min:" << nmea_time->min << "sec:" << nmea_time->sec;
    } else {
      while (true && running_.load()) {
        if (_positioning_input == nullptr) {
          AERROR << " _positioning_input uninited.";
          return;
        }

        int rc = _positioning_input->get_positioning_data_packtet(nmea_time);
        AINFO << "frame_id:" << _config.frame_id()
              << "-T(gps-time):" << nmea_time->year << "-" << nmea_time->mon
              << "-" << nmea_time->day << "  " << nmea_time->hour << "-"
              << nmea_time->min << "-" << nmea_time->sec;
        if (rc == 0) {
          break;  // got a full packet
        }
        if (rc < 0) {
          ret = false;  // end of file reached
        }
      }
    }

    if (_basetime == 0 && ret) {
      set_base_time_from_nmea_time(nmea_time, &_basetime,
                                   _config.use_gps_time());
      break;
    }
  }
}

}  // namespace surestar
}  // namespace drivers
}  // namespace apollo
