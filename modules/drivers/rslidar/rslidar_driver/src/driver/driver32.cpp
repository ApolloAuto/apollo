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

#include "driver.h"

#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <string>
#include <thread>

namespace apollo {
namespace drivers {
namespace rslidar {

Rslidar32Driver::Rslidar32Driver(const Config &config) {
  config_ = config;
}

void Rslidar32Driver::init(ros::NodeHandle &node) {
  const double packet_rate = 840;                 // packet frequency (Hz)
  const double frequency = (config_.rpm / 60.0);  // expected Hz rate

  config_.npackets = (int)ceil(packet_rate / frequency);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");


  input_.reset(new SocketInput());
  input_->init(config_.msop_data_port);

   output_ = node.advertise<rslidar_msgs::rslidarScan>(config_.topic, 10);
  
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool Rslidar32Driver::poll(void) {
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  rslidar_msgs::rslidarScanPtr scan(
      new rslidar_msgs::rslidarScan);
  int poll_result = poll_standard(scan);

  if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL) {
    return true;  // poll again
  }

  if (scan->packets.empty()) {
    ROS_INFO_STREAM("Get a empty scan from port: " << config_.msop_data_port);
    return true;
  }
  scan->header.stamp = ros::Time().now();
  scan->header.frame_id = config_.frame_id;
 
  output_.publish(scan);

  return true;
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
void Rslidar32Driver::poll_positioning_packet(void) {
  while (true) {
    NMEATimePtr nmea_time(new NMEATime);
    bool ret = true;
    if (basetime_ == 0 && ret) {
      set_base_time_from_nmea_time(nmea_time, basetime_);
    }
  }
}

}  // namespace rslidar
}  // namespace drivers
}  // namespace apollo
