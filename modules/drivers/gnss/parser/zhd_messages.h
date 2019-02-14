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
  FIX_TYPE_NONE = 0, //Fix not available or invalid
  FIX_TYPE_SPOINT =1, //Single point
  FIX_TYPE_VBS_PPP =2,// Pseudorange differential
  FIX_TYPE_RT2 =4,  //RTK fixed ambiguity solution (RT2)
  FIX_TYPE_PPP =5,  //Converged PPP
  FIX_TYPE_DEAD_MOD =6,  //Dead reckoning mode
  FIX_TYPE_INPUT_MOD =7,  //Manual input mode (fixed position)
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
  unsigned char head[2];          //2 bytes deviation 0
  unsigned short int version;     //2 bytes deviation 2
  unsigned short int length;      //2 bytes deviation 4
  unsigned short int freq;        //2 bytes deviation 6
  float time_utc;                 //4 bytes deviation 8
  unsigned short int year_utc;    //2 bytes deviation 12
  unsigned short int month_utc;   //2 bytes deviation 14
  unsigned short int day_utc;     //2 bytes deviation 16
  unsigned short int hour_utc;    //2 bytes deviation 18
  unsigned short int min_utc;     //2 bytes deviation 20
  unsigned short int sec_utc;     //2 bytes deviation 22
  double latitude;                //8 bytes deviation 24
  double longitude;               //8 bytes deviation 32
  double altitude;                //8 bytes deviation 40
  float eph;                      //4 bytes deviation 48
  float epv;                      //4 bytes deviation 52
  float vel_ground_m_s;           //4 bytes deviation 56
  float angle_tracktrue;          //4 bytes deviation 60
  float angle_heading;            //4 bytes deviation 64
  float angle_pitch;              //4 bytes deviation 68
  double vel_n_m_s;                   //8 bytes deviation 72
  double vel_e_m_s;                   //8 bytes deviation 80
  double vel_u_m_s;                   //8 bytes deviation 88
  
  unsigned short int satellites_used;   //2 bytes deviation 96
  unsigned short int satellites_track;  //2 bytes deviation 98
  float vel_ned_valid;                  //4 bytes deviation 100
  unsigned short int fix_type;          //2 bytes deviation 104
  float angle_postype;                  //4 bytes deviation 106
  float head_deviation;                 //4 bytes deviation 110

  unsigned short int ins_state;         //2 bytes deviation 114
  double gnss_alt_delta ;               //8 bytes deviation 116
  double ellipsoidal_h;                 //8 bytes deviation 124
  unsigned short int diff_age;          //2 bytes deviation 132
  unsigned char reserve[2];             //2 bytes deviation 134
  unsigned short int checksum;          //2 bytes deviation 136
}gps_rtk_zhd_packet_t; //total 138 bytes

#pragma pack()

}  // namespace zhd
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
