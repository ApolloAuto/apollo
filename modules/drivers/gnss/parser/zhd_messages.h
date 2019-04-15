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
#pragma once

#include <stdint.h>
#include "modules/drivers/gnss/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace zhd {

#pragma pack(1)

enum SyncByte : uint8_t {
  SYNC_0 = 0xAA,
  SYNC_1 = 0x33,
  VERSION_2 = 0x02,
  VERSION_3 = 0x00,
};

enum class _fixtype : uint32_t {
  FIX_TYPE_NONE = 0,
  FIX_TYPE_SPOINT = 1,
  FIX_TYPE_VBS_PPP = 2,
  FIX_TYPE_RT2 = 4,
  FIX_TYPE_PPP = 5,
  FIX_TYPE_DEAD_MOD = 6,
  FIX_TYPE_INPUT_MOD = 7,
};

enum class _PosType : uint8_t {
  POS_TYPE_NONE = 0,
  POS_TYPE_FIXEDPOS = 1,
  FIX_TYPE_FIXEDHEIGHT = 2,
  POS_TYPE_DOPPLER_VELOCITY = 8,
  POS_TYPE_SINGLE = 16,
  POS_TYPE_PSRDIFF = 17,
  POS_TYPE_WAAS = 18,
  FIX_TYPE_PROPAGATED = 19,
  POS_TYPE_OMNISTAR = 20,
  POS_TYPE_L1_FLOAT = 32,
  POS_TYPE_IONOFREE_FLOAT = 33,
  POS_TYPE_NARROW_FLOAT = 34,
  FIX_TYPE_L1_INT = 48,
  POS_TYPE_NARROW_INT = 50,
  POS_TYPE_OMNISTAR_HP = 64,
  POS_TYPE_OMNISTAR_XP = 65,
  POS_TYPE_PPP_CONVERGING = 68,
  POS_TYPE_PPP = 69,
  POS_TYPE_INS_Angle_state = 99,
};

typedef struct {
  unsigned char head[2];
  uint16_t version;
  uint16_t length;
  uint16_t freq;
  float time_utc;
  uint16_t year_utc;
  uint16_t month_utc;
  uint16_t day_utc;
  uint16_t hour_utc;
  uint16_t min_utc;
  uint16_t sec_utc;
  double latitude;
  double longitude;
  double altitude;
  float eph;
  float epv;
  float vel_ground_m_s;
  float angle_tracktrue;
  float angle_heading;
  float angle_pitch;
  double vel_n_m_s;
  double vel_e_m_s;
  double vel_u_m_s;

  uint16_t satellites_used;
  uint16_t satellites_track;
  float vel_ned_valid;
  uint16_t fix_type;
  float angle_postype;
  float head_deviation;

  uint16_t ins_state;
  double gnss_alt_delta;
  double ellipsoidal_h;
  uint16_t diff_age;
  unsigned char reserve[2];
  uint16_t checksum;
} gps_rtk_zhd_packet_t;

#pragma pack()

}  // namespace zhd
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
