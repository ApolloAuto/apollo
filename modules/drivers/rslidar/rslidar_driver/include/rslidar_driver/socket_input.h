/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_DRIVERS_ROBOSENSE_RSLIDAR_DRIVER_SOCKET_INPUT_H_
#define MODULES_DRIVERS_ROBOSENSE_RSLIDAR_DRIVER_SOCKET_INPUT_H_

#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>

// #include "roslibmetric/metric_handle.h"

#include "input.h"

namespace apollo {
namespace drivers {
namespace rslidar {

static int MSOP_DATA_PORT = 6699;//6699;//2368;
static int DIFOP_DATA_PORT = 7788;//6699;//8308;
static const int POLL_TIMEOUT = 1000;  // one second (in msec)

/** @brief Live rslidar input from socket. */
class SocketInput : public Input {
 public:
  SocketInput();
  virtual ~SocketInput();
  void init(int &port);
  int get_msop_data_packet(rslidar_msgs::rslidarPacket *pkt);
  //int get_positioning_data_packtet(const NMEATimePtr &nmea_time);

 private:
  int sockfd_;
  int port_;
  bool input_available(int timeout);
};

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo

#endif  
