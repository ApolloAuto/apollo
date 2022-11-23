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

#pragma once
#include <unistd.h>
#include <cstdio>

#include <pcap.h>

#include "modules/drivers/lidar/robosense/lib/data_type.h"
#include "modules/drivers/lidar/robosense/lib/input.h"

namespace apollo {
namespace drivers {
namespace robosense {

/** @brief Live suteng input from socket. */
class SocketInput : public Input {
 public:
  SocketInput();
  virtual ~SocketInput();
  void init(uint32_t port);
  int get_firing_data_packet(apollo::drivers::suteng::SutengPacket* pkt,
                             int time_zone, uint64_t start_time_);
  int get_positioning_data_packtet(const NMEATimePtr& nmea_time);

 private:
  int sockfd_;
  int port_;
  bool input_available(int timeout);
};
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
