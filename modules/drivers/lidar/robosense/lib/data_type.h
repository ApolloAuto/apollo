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
#include <stdint.h>

#include <limits>
#include <memory>
#include <vector>
namespace apollo {
namespace drivers {
namespace robosense {

/**
 * Raw suteng packet constants and structures.
 */
// static uint64_t pkts_stamp_[84]={0};

static const int BLOCK_SIZE = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;  // 1block = 32 points
static const int BLOCK_DATA_SIZE =
    (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // size of a block

static const float ROTATION_RESOLUTION = 0.01f; /**< degrees */
// static const uint16_t ROTATION_MAX_UNITS = 36000; [>*< hundredths of degrees
// <]
// because angle_rang is [0, 36000], so thie size is 36001
static const uint16_t ROTATION_MAX_UNITS = 36001; /**< hundredths of degrees */

/** According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
 *  valid packets with readings up to 130.0. */
static const float DISTANCE_MAX = 130.0f;                        /**< meters */
static const float DISTANCE_RESOLUTION = 0.005f; /**< meters */  // rs16
static const float DISTANCE_MAX_UNITS =
    (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

// laser_block_id
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

static const float ANGULAR_RESOLUTION = 0.00300919;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;
static const float VLP16_DSR_TOFFSET = 2.304f;
static const float VLP16_FIRING_TOFFSET = 55.296f;

static const int PACKET_SIZE = 1248;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET =
    (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);  // 1 packet的point 个数

static constexpr uint32_t VLP16_SCAN_SIZE =
    80;  // ceil 754/10  一圈的packet的数目
static constexpr uint32_t VLP16_POINT_SIZE =
    28800;  // 32256;//VLP16_SCAN_SIZE * SCANS_PER_PACKET; //转一圈点的个数
static constexpr uint32_t HDL64S3D_SCAN_SIZE = 579;  // ceil 5789/10
static constexpr uint32_t HDL64S3D_POINT_SIZE =
    HDL64S3D_SCAN_SIZE * SCANS_PER_PACKET;

/** \brief Raw suteng data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
struct RawBlock {
  uint16_t laser_block_id;        ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;              ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];  // 32*3
};

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union RawDistance {
  uint16_t raw_distance;
  uint8_t bytes[2];
};

enum StatusType {
  HOURS = 72,
  MINUTES = 77,
  SECONDS = 83,
  DATE = 68,
  MONTH = 78,
  YEAR = 89,
  GPS_STATUS = 71
};

/** \brief Raw suteng packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  status has either a temperature encoding or the microcode level
 */
struct RawPacket {
  RawBlock blocks[BLOCKS_PER_PACKET];
  unsigned int gps_timestamp;
  unsigned char status_type;
  unsigned char status_value;
};

enum Mode { STRONGEST, LAST, DUAL };

static const float nan = std::numeric_limits<float>::signaling_NaN();

/** \brief Raw suteng packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  status has either a temperature encoding or the microcode level
 */

// suteng1
struct RslidarPic {
  uint32_t col;
  std::vector<float> distance;
  std::vector<float> intensity;
  std::vector<float> azimuthforeachP;
  std::vector<uint64_t> timestamp;
};

// Special Defines for RS16 support
static const int RS16_FIRINGS_PER_BLOCK = 2;
static const int RS16_SCANS_PER_FIRING = 16;
static const float RS16_BLOCK_TDURATION = 100.0f;  // [µs]
static const float RS16_DSR_TOFFSET = 3.0f;        // [µs]
static const float RS16_FIRING_TOFFSET = 50.0f;    // [µs]
static const int RS16_DATA_NUMBER_PER_SCAN =
    40000;  // Set 40000 to be large enough

typedef struct raw_block {
  uint16_t header;  // UPPER_BANK or LOWER_BANK
  uint8_t rotation_1;
  uint8_t rotation_2;             // combine r1 and r2 together to get 0-35999
                                  // divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];  // 96
} raw_block_t;

typedef struct raw_packet {
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint8_t unused[4];
  uint16_t flag;
} raw_packet_t;

union two_bytes {
  uint16_t uint;
  uint8_t bytes[2];
};
// suteng2
union UPacket {
  struct {
    RawBlock blocks[BLOCKS_PER_PACKET];
    unsigned int gps_timestamp;
    unsigned char status_type;
    unsigned char status_value;
  };
  uint8_t data[PACKET_SIZE];
};
typedef std::shared_ptr<UPacket> UPacketPtr;

struct NMEATime {
  uint16_t year;
  uint16_t mon;
  uint16_t day;
  uint16_t hour;
  uint16_t min;
  uint16_t sec;
  uint16_t msec;
  uint16_t usec;
};
typedef std::shared_ptr<NMEATime> NMEATimePtr;
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
