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

#ifndef DRIVERS_SURESTAR_INCLUDE_LIB_INPUT_H
#define DRIVERS_SURESTAR_INCLUDE_LIB_INPUT_H

#include <stdio.h>
#include <unistd.h>

#include <pcap.h>

#include "cyber/cyber.h"
#include "modules/drivers/lidar_surestar/lib/data_type.h"
#include "modules/drivers/lidar_surestar/proto/sensor_surestar.pb.h"

namespace apollo {
namespace drivers {
namespace surestar {

static const size_t FIRING_DATA_PACKET_SIZE = 1206;
static const size_t POSITIONING_DATA_PACKET_SIZE = 256;  // beike-256
static const size_t ETHERNET_HEADER_SIZE = 42;
static const int PCAP_FILE_END = -1;
static const int SOCKET_TIMEOUT = -2;
static const int RECIEVE_FAIL = -3;

// beike ------------------------------begin
#pragma pack(1)
typedef struct {
  unsigned int pkgflag;
  unsigned int pkgnumber;
  unsigned int date;
  uint16_t time;
  unsigned int maca;
  uint16_t macb;
  uint16_t dataport;
  uint16_t msgport;
  unsigned char motorspd;
  unsigned int deviceType;
  uint16_t phaseAngle;
  unsigned char padding[225];
} RFANS_HEARTBEAT_S;
#pragma pack()
// beike ------------------------------end

/** @brief Pure virtual surestar input base class */
class Input {
 public:
  Input() {}
  virtual ~Input() {}

  /** @brief Read one surestar packet.
   *
   * @param pkt points to SurestarPacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int get_firing_data_packet(
      apollo::drivers::Surestar::SurestarPacket* pkt) = 0;
  virtual int get_positioning_data_packtet(const NMEATimePtr& nmea_time) = 0;
  virtual void init() {}
  virtual void init(uint32_t port) { (void)port; }

 protected:
  bool exract_nmea_time_from_packet(const NMEATimePtr& nmea_time,
                                    const uint8_t* bytes);
};
}  // namespace surestar
}  // namespace drivers
}  // namespace apollo

#endif  // DRIVERS_SURESTAR_INCLUDE_LIB_INPUT_H
