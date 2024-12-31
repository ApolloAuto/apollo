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

// This defines enums and structures for parsing ENS binary messages. Please
// refer to ENS's
// documents for details about these messages.

#pragma once

#include <cstdint>
#include <limits>

#include "modules/drivers/gnss/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace enbroad {

enum EFUSHIONSTATUS {
  E_FUNSION_NONE = 0,
  E_FUNSION_GPS = 1,
  E_FUNSION_WHEEL = 2,
  E_FUNSION_MOTION = 3,
  E_FUNSION_TOTAL = 4,
};

enum ENAVSTANDARDFLAGSTATUS {
  E_NAV_STANDARD_NO_PROCCSS = 0,
  E_NAV_STANDARD_PROCCSSING = 1,
  E_NAV_STANDARD_PROCCSSED = 2,
  E_NAV_STANDARD_TOTAL = 3,
};

enum ENAVSTATUS {
  E_NAV_STATUS_START = 0,
  E_NAV_STATUS_ROUTH_ALIGN = 1,
  E_NAV_STATUS_SINS_KF_INITIAL = 2,
  E_NAV_STATUS_SYSTEM_STANDARD = 3,
  E_NAV_STATUS_IN_NAV = 4,
  E_NAV_STATUS_STOP = 5,
  E_NAV_STATUS_TOTAL = 6,
};

enum EGPSRTKSTATUS {
  E_GPS_RTK_INVALID = 0,
  E_GPS_RTK_SPP = 1,
  E_GPS_RTK_DGPS = 2,
  E_GPS_RTK_FIXED = 4,
  E_GPS_RTK_FLOAT = 5,
  E_GPS_RTK_TOTAL = 6,
};

enum EPOLLDATATYPE {
  E_POLL_DEV_TEMP = 0,
  E_POLL_GNSS_STATE = 20,
  E_POLL_IMU_STATE = 40,
  E_POLL_CAN_STATE = 60,
  E_POLL_INS_STANDARD_GNSS2 = 90,
  E_POLL_INS_STANDARD_ODS2 = 95,
  E_POLL_INS_STATE = 80,
  E_POLL_GNSS2_STATE = 100,
  E_POLL_DATA_TOTAL = 120,
};

enum MessageId : uint16_t {
  BIN_NAV_DATA = 0x01AA,
  BIN_SINS_DATA = 0x02AA,
  BIN_IMU_DATA = 0x03AA,
  BIN_GNSS_DATA = 0x04AA,
  BIN_Extend_DATA = 0x05AA,
};

// Every binary message has 32-bit CRC performed on all data including the
// header.

#pragma pack(push, 1)  // Turn off struct padding.

enum class DatumId : uint32_t {
  // We only use WGS-84.
  WGS84 = 61,
};

enum SyncHeadByte : uint8_t {
  SYNC_HEAD_0 = 0xA5,
  SYNC_HEAD_1 = 0x5A,
  SYNC_HEAD_2 = 0x5A,
};

enum SyncTailByte : uint8_t {
  SYNC_TAIL_0 = 0xA5,
  SYNC_Tail_1 = 0x5A,
  SYNC_Tail_2 = 0x5A,
};

struct FrameHeader {
  SyncHeadByte sync_head[3];
  uint8_t src_id;
  uint8_t dst_id;
  MessageId message_id;
  uint8_t rsp;
  uint16_t SN;
  uint16_t reserved;
  uint16_t message_length;
};
static_assert(sizeof(FrameHeader) == 14, "Incorrect FrameHeader");
struct NAV_DATA_TypeDef {
  uint32_t gps_week;       // GPS Week number.
  uint32_t gps_millisecs;  // Milliseconds of week.
  int16_t pitch;
  int16_t roll;
  int16_t head;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t magnetX;
  int16_t magnetY;
  int16_t magentZ;
  int64_t lat;
  int64_t lon;
  int32_t alt;
  int16_t ve;
  int16_t vn;
  int16_t vu;
  uint8_t poll_type;
  int16_t poll_frame1;
  int16_t poll_frame2;
  int16_t poll_frame3;
};
// static_assert(sizeof(NAV_DATA_TypeDef) == 65, "Incorrect NAV_DATA_TypeDef");

struct NAV_SINS_TypeDef {
  uint32_t gps_week;   // GPS Week number.
  uint32_t gpssecond;  // Milliseconds of week.
  uint8_t navStatus;
  uint8_t fusion;  // 0:valid;1:gps;2:wheel;3:motion
  float pitch;
  float roll;          // unit degree
  float heading;       // unit degree
  float ve;            // East velocity, unit m/s
  float vn;            // North velocity,unit m/s
  float vu;            // up velocity,unit m/s
  double latitude;     // unit degree
  double longitude;    // unit degree
  float altitude;      // unit m
  float xigema_ve;     // East velocity std unit m/s
  float xigema_vn;     // North velocity std unit m/s
  float xigema_vu;     // up velocity std unit m/s
  float xigema_lat;    // North pos std unit m
  float xigema_lon;    // East pos std unit m
  float xigema_alt;    // up pos std unit m
  float xigema_pitch;  // pitch std unit degree
  float xigema_roll;   // roll std unit degree
  float xigema_head;   // heading std unit degree
};
// static_assert(sizeof(NAV_SINS_TypeDef) == 90, "Incorrect NAV_SINS_TypeDef");

struct NAV_IMU_TypeDef {
  uint32_t gps_week;   // GPS Week number.
  uint32_t gpssecond;  // Milliseconds of week.
  float sensorTemp;    // unit degree
  uint8_t gyroFlag;    // gyro state,0:abnormal,1:normal
  double gyroX;        // unit degree/s
  double gyroY;        // unit degree/s
  double gyroZ;        // unit degree/s
  uint8_t accFlag;     // acc state,0:abnormal,1:normal
  double accX;         // unit m/s2
  double accY;         // unit m/s2
  double accZ;         // unit m/s2
  uint8_t magnetFlag;  // magnet state,0:abnormal,1:normal
  double magnetX;      // unit mGauss
  double magnetY;      // unit mGauss
  double magnetZ;      // unit mGauss
};
// static_assert(sizeof(NAV_IMU_TypeDef) == 87, "Incorrect NAV_IMU_TypeDef");

struct NAV_GNSS_TypeDef {
  uint32_t gps_week;   // GPS Week number.
  uint32_t gpssecond;  // Milliseconds of week.
  uint8_t satsNum;     // num of sats
  float age;
  uint8_t rtkStatus;      // 0:invalid;1:SPP;2:DGPS;4:FIXED;5:FLOAT
  double latitude;        // unit degree
  double longitude;       // unit degree
  float altitude;         // unit m
  uint8_t headingStatus;  // 0:invalid;4:FIXED;5:FLOAT
  float baseline;         // unit m
  float heading;          // unit degree
  uint8_t velStatus;      // 0:normal
  float ve;               // East velocity, unit m/s
  float vn;               // North velocity,unit m/s
  float vu;               // up velocity,unit m/s
  float xigema_ve;        // East velocity std unit m/s
  float xigema_vn;        // North velocity std unit m/s
  float xigema_vu;        // up velocity std unit m/s
  float xigema_lat;       // North pos std unit m
  float xigema_lon;       // East pos std unit m
  float xigema_alt;       // up pos std unit m
  float xigema_heading;   // heading std unit degree
};
// static_assert(sizeof(NAV_GNSS_TypeDef) == 84, "Incorrect NAV_GNSS_TypeDef");

struct NAV_Extend_TypeDef {
  uint32_t gps_week;   // GPS Week number.
  uint32_t gpssecond;  // Milliseconds of week.
  float corrGyroX;
  float corrGyroY;
  float corrGyroZ;
  float corrAccX;
  float corrAccY;
  float corrAccZ;
  float gnssAttFromIMU;  // GNSS angle precision compensation from IMU
  float odsAttFromIMU;   // Carrier angle compensation from IMU
  float Undulation;      //  Geoidal separation
};
// static_assert(sizeof(NAV_Extend_TypeDef) == 44, "Incorrect
// NAV_Extend_TypeDef");

#pragma pack(pop)

}  // namespace enbroad
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
