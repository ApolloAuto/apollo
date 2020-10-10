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

#pragma once

#include <unistd.h>
#include <cstdio>

#include "modules/drivers/velodyne/driver/input.h"

namespace apollo {
namespace drivers {
namespace velodyne {

using apollo::drivers::velodyne::VelodynePacket;

// static int FIRING_DATA_PORT = 2368;
// static int POSITIONING_DATA_PORT = 8308;
static const int POLL_TIMEOUT = 1000;  // one second (in msec)

/** @brief Live Velodyne input from socket. */
class SocketInput : public Input {
 public:
  SocketInput();
  virtual ~SocketInput();
  void init(const int& port) override;
  int get_firing_data_packet(VelodynePacket* pkt);
  int get_positioning_data_packet(NMEATimePtr nmea_time);

 private:
  int sockfd_;
  int port_;
  bool input_available(int timeout);
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
