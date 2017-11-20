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

#include "driver.h"

#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <string>

namespace apollo {
namespace drivers {
namespace velodyne {

Velodyne64Driver::Velodyne64Driver(Config config) {
  config_ = config;
}

void Velodyne64Driver::init(ros::NodeHandle &node) {
  double packet_rate = 0;  // packet frequency (Hz)
  if (config_.model == "64E_S2" || config_.model == "64E_S3S") {
    packet_rate = 3472.17;  // 1333312 / 384
  } else {                  // 64E_S3D etc.
    packet_rate = 5789;
  }
  double frequency = config_.rpm / 60.0;  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = static_cast<int>(ceil(packet_rate / frequency));
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  input_.reset(new SocketInput());
  input_->init(config_.firing_data_port);

  // raw data output topic
  output_ =
      node.advertise<velodyne_msgs::VelodyneScanUnified>(config_.topic, 10);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool Velodyne64Driver::poll(void) {
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanUnifiedPtr scan(
      new velodyne_msgs::VelodyneScanUnified());

  int poll_result = poll_standard(scan);

  if (poll_result == SOCKET_TIMEOUT || poll_result == RECIEVE_FAIL) {
    return true;  // poll again
  }

  if (scan->packets.empty()) {
    ROS_INFO_STREAM("Get a empty scan from port: " << config_.firing_data_port);
    return true;
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.stamp = ros::Time().now();
  scan->header.frame_id = config_.frame_id;
  scan->basetime = basetime_;
  output_.publish(scan);

  return true;
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
