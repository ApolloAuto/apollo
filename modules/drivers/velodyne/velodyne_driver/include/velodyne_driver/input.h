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

#ifndef MODULES_DRIVERS_VELODYNE_VELODYNE_DRIVER_INPUT_H_
#define MODULES_DRIVERS_VELODYNE_VELODYNE_DRIVER_INPUT_H_

#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>

#include "velodyne_msgs/VelodyneScanUnified.h"

namespace apollo {
namespace drivers {
namespace velodyne {

static const size_t FIRING_DATA_PACKET_SIZE = 1206;
static const size_t POSITIONING_DATA_PACKET_SIZE = 512;
static const size_t ETHERNET_HEADER_SIZE = 42;
static const int SOCKET_TIMEOUT = -2;
static const int RECIEVE_FAIL = -3;

struct NMEATime {
  uint16_t year;
  uint16_t mon;
  uint16_t day;
  uint16_t hour;
  uint16_t min;
  uint16_t sec;
};
typedef boost::shared_ptr<NMEATime> NMEATimePtr;

/** @brief Pure virtual Velodyne input base class */
class Input {
 public:
  Input() {}
  virtual ~Input() {}

  /** @brief Read one Velodyne packet.
   *
   * @param pkt points to VelodynePacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int get_firing_data_packet(velodyne_msgs::VelodynePacket* pkt) = 0;
  virtual int get_positioning_data_packtet(const NMEATimePtr& nmea_time) = 0;
  virtual void init() {}
  virtual void init(int& port) {}

 protected:
  bool exract_nmea_time_from_packet(const NMEATimePtr& nmea_time,
                                    const uint8_t* bytes);
};

}  // namespace velodyne
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_VELODYNE_VELODYNE_DRIVER_INPUT_H_
