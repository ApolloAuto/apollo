/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#ifndef LIDAR_HESAI_SRC_TYPE_DEFS_H_
#define LIDAR_HESAI_SRC_TYPE_DEFS_H_

#include "modules/drivers/hesai/const_var.h"

namespace apollo {
namespace drivers {
namespace hesai {

typedef struct HesaiPacket {
  double stamp;
  uint8_t data[ETHERNET_MTU];
  uint32_t size;
} HesaiPacket;

/**************Hesai40P****************************/
typedef struct Hesai40PUnit {
  uint8_t intensity;
  double distance;
} Hesai40PUnit;

typedef struct Hesai40PBlock {
  uint16_t azimuth;
  uint16_t sob;
  Hesai40PUnit units[LASER_COUNT];
} Hesai40PBlock;

typedef struct Hesai40Packet {
  Hesai40PBlock blocks[BLOCKS_PER_PACKET];
  struct tm t;
  uint32_t usec;
  int echo;
} Hesai40Packet;

/************Hesai64*******************************/
typedef struct Hesai64Header {
  uint16_t sob;     // 0xFFEE 2bytes
  uint8_t chLaserNumber;  // laser number 1byte
  uint8_t chBlockNumber;  // block number 1byte
  uint8_t chReturnType;   // return mode 1 byte
                          // when dual return 0
                          // Single Return
                          // 1-The first block is the 1 st return.
                          // 2-The first block is the 2 nd return
  uint8_t chDisUnit;      // Distance unit, 6mm/5mm/4mm
 public:
  Hesai64Header() {
    sob = 0;
    chLaserNumber = 0;
    chBlockNumber = 0;
    chReturnType = 0;
    chDisUnit = 0;
  }
} Hesai64Header;

typedef struct Hesai64Unit {
  double distance;
  uint8_t reflectivity;  // reflectivity
} Hesai64Unit;

typedef struct Hesai64Block {
  uint16_t azimuth;  // packet angle  ,Azimuth = RealAzimuth * 100
  Hesai64Unit units[LASER_COUNT_L64];
} Hesai64Block;

typedef struct Hesai64Packet {
  Hesai64Header header;
  Hesai64Block blocks[BLOCKS_PER_PACKET_L64];
  uint8_t echo;
  unsigned int timestamp;  // ms
  unsigned char utc_time[UTC_TIME];
} Hesai64Packet;

/***************GPS****************************/
typedef struct GPS {
  uint16_t flag;
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t second;
  uint16_t minute;
  uint16_t hour;
  uint32_t fineTime;
} Gps;

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif
