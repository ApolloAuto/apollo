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

#include "modules/drivers/lidar/robosense/driver/driver16p.h"

namespace apollo {
namespace drivers {
namespace robosense {

Robosense16PDriver::Robosense16PDriver(
    const apollo::drivers::suteng::SutengConfig& robo_config)
    : RobosenseDriver() {
  AINFO << "Robosense16PDriver HELIOS_16P 0";
  config_ = robo_config;
}

Robosense16PDriver::~Robosense16PDriver() {
  running_.store(false);
  if (positioning_thread_.joinable()) {
    positioning_thread_.join();
  }
}

void Robosense16PDriver::init() {
  running_.store(true);
  double packet_rate =
      750;  // 840.0;          //每秒packet的数目   packet frequency (Hz)
            // robosense-754  beike v6k-781.25  v6c-833.33
  double frequency = (config_.rpm() / 60.0);  //  每秒的圈数  expected Hz rate

  AINFO << "Robosense16PDriver HELIOS_16P 1";
  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.set_npackets(
      ceil(packet_rate / frequency));  // 每frame，即转一圈packet的数目
  AINFO << "Time Synchronized is using time of pakcet";
  AINFO << "_config.npackets() == " << config_.npackets();
  if (config_.has_pcap_file()) {
    AWARN << "Helios 16P does not support pcap_file. Using Socket instead.";
  }

  input_16p_.reset(new robosense::SocketInput16P());
  input_16p_->init(config_.firing_data_port());

  positioning_input_.reset(new robosense::SocketInput16P());
  positioning_input_->init(config_.positioning_data_port());

  positioning_thread_ =
      std::thread(&Robosense16PDriver::poll_positioning_packet, this);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool Robosense16PDriver::poll(
    const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan) {
  int poll_result = SOCKET_TIMEOUT;
  auto t0 = apollo::cyber::Time().Now().ToNanosecond();
  // Synchronizing all laser radars
  poll_result = poll_msop_sync_count(scan);

  AINFO << "poll time: " << apollo::cyber::Time().Now().ToNanosecond()
        << " cost: " << (apollo::cyber::Time().Now().ToNanosecond() - t0) * 1e-6
        << "ms";
  if (poll_result == PCAP_FILE_END) {
    return false;  // read the end of pcap file, return false stop poll;
  }

  if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL) {
    return false;  // poll again
  }

  if (scan->firing_pkts_size() == 0) {
    AINFO << "Get an empty scan from port: " << config_.firing_data_port();
    return false;
  }

  scan->set_model(config_.model());
  scan->set_mode(config_.mode());
  scan->mutable_header()->set_frame_id(config_.frame_id());
  scan->mutable_header()->set_lidar_timestamp(basetime_);

  if (positioning_pkts_.IsInitialized()) {
    auto pkt = scan->add_positioning_pkts();
    pkt->CopyFrom(positioning_pkts_);
  }

  scan->set_basetime(basetime_);

  return true;
}

int Robosense16PDriver::poll_msop_sync_count(
    const std::shared_ptr<apollo::drivers::suteng::SutengScan>& scan) {
  static std::atomic_ullong sync_counter(0);
  if (config_.main_frame()) {
    for (int32_t i = 0; i < config_.npackets(); ++i) {
      while (true) {
        apollo::drivers::suteng::SutengPacket* packet;
        // keep reading until full packet received
        packet = scan->add_firing_pkts();
        int rc =
            input_16p_->get_firing_data_packet(packet, config_.use_gps_time());
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
        int rc =
            input_16p_->get_firing_data_packet(packet, config_.use_gps_time());
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

/** poll the device
 *
 *  @returns true unless end of file reached
 */
void Robosense16PDriver::poll_positioning_packet(void) {
  while (!apollo::cyber::IsShutdown() && running_.load()) {
    if (positioning_input_ == nullptr) {
      AERROR << " positioning_input_ uninited.";
      return;
    }
    int rc = positioning_input_->get_positioning_data_packet(
        &positioning_pkts_, config_.use_gps_time());

    AINFO << "Difop data rc: " << rc << "---rc is normal when it is 0";
  }
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
