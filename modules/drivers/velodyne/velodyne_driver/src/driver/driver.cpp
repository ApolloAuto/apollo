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

VelodyneDriver::VelodyneDriver() : _basetime(0), _last_gps_time(0) {}

int VelodyneDriver::poll_standard(velodyne_msgs::VelodyneScanUnifiedPtr& scan) {
  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  scan->packets.resize(_config.npackets);
  for (int i = 0; i < _config.npackets; ++i) {
    while (true) {
      // keep reading until full packet received
      int rc = _input->get_firing_data_packet(&scan->packets[i]);

      if (rc == 0) {
        break;  // got a full packet?
      }

      if (rc < 0) {
        return rc;
      }
    }
  }

  return 0;
}

VelodyneDriver* VelodyneDriverFactory::create_driver(
    ros::NodeHandle private_nh) {
  Config config;
  // use private node handle to get parameters
  private_nh.param("frame_id", config.frame_id, std::string("velodyne64"));
  private_nh.param("model", config.model, std::string("64E"));
  private_nh.param("topic", config.topic, std::string("velodyne_packets"));
  private_nh.param("firing_data_port", config.firing_data_port,
                   FIRING_DATA_PORT);
  private_nh.param("rpm", config.rpm, 600.0);

  if (config.model == "64E_S2" || config.model == "64E_S3S" ||
      config.model == "64E_S3D_STRONGEST" || config.model == "64E_S3D_LAST" ||
      config.model == "64E_S3D_DUAL") {
    return new Velodyne64Driver(config);
  } else {
    ROS_ERROR_STREAM("invalid model, must be 64E_S2|64E_S3S"
                     << "|64E_S3D_STRONGEST|64E_S3D_LAST|64E_S3D_DUAL");
    return nullptr;
  }
}

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo
