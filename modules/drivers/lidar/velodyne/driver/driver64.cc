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

#include "modules/common/util/message_util.h"
#include "modules/drivers/lidar/velodyne/driver/driver.h"
// #include "ros/ros.h"

namespace apollo {
namespace drivers {
namespace velodyne {
Velodyne64Driver::~Velodyne64Driver() {
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
}

bool Velodyne64Driver::Init() {
  const double frequency = config_.rpm() / 60.0;  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.set_npackets(static_cast<int>(ceil(packet_rate_ / frequency)));
  AINFO << "publishing " << config_.npackets() << " packets per scan";

  input_.reset(new SocketInput());
  input_->init(config_.firing_data_port());

  if (node_ == NULL) {
    AERROR << "node is NULL";
    return false;
  }
  writer_ = node_->CreateWriter<VelodyneScan>(config_.scan_channel());
  poll_thread_ = std::thread(&Velodyne64Driver::DevicePoll, this);
  return true;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool Velodyne64Driver::Poll(const std::shared_ptr<VelodyneScan>& scan) {
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  int poll_result =
      config_.use_sensor_sync() ? PollStandardSync(scan) : PollStandard(scan);

  if (poll_result == SOCKET_TIMEOUT || poll_result == RECEIVE_FAIL) {
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
  scan->set_basetime(basetime_);

  return true;
}

bool Velodyne64Driver::CheckAngle(const VelodynePacket& packet) {
  // check the angle in every packet
  // for each model of velodyne 64 the data struct is same , so we don't need to
  // check the lidar model
  const unsigned char* raw_ptr = (const unsigned char*)packet.data().c_str();
  for (int i = 0; i < BLOCKS_PER_PACKET; ++i) {
    uint16_t angle = static_cast<uint16_t>(raw_ptr[i * BLOCK_SIZE + 3] * 256 +
                                           raw_ptr[i * BLOCK_SIZE + 2]);
    // for the velodyne64 angle resolution is 0.17~0.2 , so take the angle diff
    // at 0.3 degree should be a good choice
    // prefix_angle default = 18000
    if (angle > config_.prefix_angle() &&
        std::abs(angle - config_.prefix_angle()) < 30) {
      return true;
    }
  }
  return false;
}

int Velodyne64Driver::PollStandardSync(std::shared_ptr<VelodyneScan> scan) {
  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  while (true) {
    while (true) {
      // keep reading until full packet received
      VelodynePacket* packet = scan->add_firing_pkts();
      int rc = input_->get_firing_data_packet(packet);

      if (rc == 0) {
        // check the angle for every packet if a packet has an angle
        if (CheckAngle(*packet) &&
            (scan->firing_pkts_size() > 0.5 * config_.npackets())) {
          return 0;
        } else {
          break;  // got a full packet?
        }
      }
      if (rc < 0) {
        return rc;
      }
    }
    // if the only UDP packet lost then recv 1.5*config_.npackets  packets at
    // most
    if (scan->firing_pkts_size() > 1.5 * config_.npackets()) {
      return 0;
    }
  }
  return 0;
}

void Velodyne64Driver::DevicePoll() {
  while (!apollo::cyber::IsShutdown()) {
    // poll device until end of file
    std::shared_ptr<VelodyneScan> scan = std::make_shared<VelodyneScan>();
    bool ret = Poll(scan);
    if (ret) {
      apollo::common::util::FillHeader("velodyne", scan.get());
      writer_->Write(scan);
    } else {
      AWARN << "device poll failed";
    }
  }

  AERROR << "CompVelodyneDriver thread exit";
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
