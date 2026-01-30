/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <cmath>
#include <iostream>

#include "modules/drivers/gnss/parser/asensing_parser/protocol_asensing.h"
#include "modules/drivers/gnss/parser/parser_common.h"

typedef struct { /* time struct */
  time_t time;   /* time (s) expressed by standard time_t */
  double sec;    /* fraction of second under 1 s */
} gtime_t;

gtime_t epoch2time_sdk(const double* ep) {
  const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
  gtime_t time = {0};
  int days, sec, year = static_cast<int>(ep[0]), mon = static_cast<int>(ep[1]),
                 day = static_cast<int>(ep[2]);

  if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

  /* leap year if year%4==0 in 1901-2099 */
  days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 +
         (year % 4 == 0 && mon >= 3 ? 1 : 0);
  sec = static_cast<int>(floor(ep[5]));
  time.time = static_cast<time_t>(days) * 86400 +
              static_cast<int>(ep[3]) * 3600 + static_cast<int>(ep[4]) * 60 +
              sec;
  time.sec = ep[5] - sec;
  return time;
}

gtime_t gpst2time_sdk(int week, double sec) {
  static const double gpst0[] = {1980, 1, 6, 0, 0, 0};
  gtime_t t = epoch2time_sdk(gpst0);

  if (sec < -1E9 || 1E9 < sec) sec = 0.0;
  t.time += 86400 * 7 * week + static_cast<int>(sec);
  t.sec = sec - static_cast<int>(sec);
  return t;
}

Decode_0B::Decode_0B() {
  filename2 = "BDDB10.csv";
  content2 =
      "Lon_deg,LonStd,Lat_deg,LatStd,Alt_m,AltStd,fix,RtkAge_s,Trk_deg,Vspd_"
      "mps,LatencyVel_s,BaseLineLenth_m,Heading_dg,HeadingStd,Pitch_deg,"
      "PitchStd,UtcYear,UtcMon,UtcDay,UtcHour,UtcMin,UtcSec,ItowPos,ItowVel,"
      "ItowHeading,RecMsgOutput,SvNumMst";

  filename0b = "BDDB0B.csv";
  content0b =
      "roll,pitch,yaw,GyroX,GyroY,GyroZ,x_acc,y_acc,z_acc,latitude,longitude,"
      "altitude,north_vel,east_vel,ground_vel,ins_status,modestatus,system_ms";

  // createFileAndWrite(filename2, content2);
  // createFileAndWrite(filename0b, content0b);

  registProtocol(m_typeImu, m_lengthImu, this);
  registProtocol(m_typeGnss, m_lengthGnss, this);
}

Decode_0B::~Decode_0B() {}

void Decode_0B::subData(const uint8_t* sub_address, int& index) {
  std::string type = "";
  char str[16] = {0};
  std::snprintf(str, sizeof(str), "%02X%02X%02X", sub_address[0],
                sub_address[1], sub_address[2]);
  type += str;
  if (type == m_typeImu) {
    parse0B(sub_address, index);
  } else if (type == m_typeGnss) {
    parseGnss(sub_address, index);
  } else {
    index += 3;
    printf("protocol type error:%s!\n", type.c_str());
  }
}

void Decode_0B::parse0B(const uint8_t* data, int& pos) {
  static const double deg_coefficient = 360.0 / 32768;
  static const double angular_coefficient = 300.0 / 32768;
  static const double acc_coefficient = 12.0 / 32768 * 9.7883105;
  static const double velocity_coefficient = 100.0 / 32768;
  static const double deg_to_rad = M_PI / 180.0;

  int sub_index = 3;
  uint8_t check_sum = 0;
  int dataLength = getLength(m_typeImu);
  /* check xor */
  for (int i = 0; i < dataLength - 1; ++i) {
    check_sum ^= data[i];
  }

  if (check_sum == data[dataLength - 1]) {
    /* roll pitch yaw */
    float roll = (toValue<int16_t>(data, sub_index)) * deg_coefficient;
    float pitch = (toValue<int16_t>(data, sub_index)) * deg_coefficient;
    float x = (toValue<int16_t>(data, sub_index)) * deg_coefficient;
    if (x < 0) {
      x = 360.0 + x;
    }
    float yaw = x;

    /* gx gy gz , ax ay az */
    double imu_msg_y_angular_velocity =
        (toValue<int16_t>(data, sub_index)) * angular_coefficient;
    double imu_msg_x_angular_velocity =
        (toValue<int16_t>(data, sub_index)) * angular_coefficient;
    double imu_msg_z_angular_velocity =
        -(toValue<int16_t>(data, sub_index)) * angular_coefficient;
    double imu_msg_y_acc =
        (toValue<int16_t>(data, sub_index)) * acc_coefficient;
    double imu_msg_x_acc =
        (toValue<int16_t>(data, sub_index)) * acc_coefficient;
    double imu_msg_z_acc =
        -(toValue<int16_t>(data, sub_index)) * acc_coefficient;

    double imu_msg_latitude = (toValue<int32_t>(data, sub_index)) * 0.0000001;
    double imu_msg_longitude = (toValue<int32_t>(data, sub_index)) * 0.0000001;
    double imu_msg_altitude = (toValue<int32_t>(data, sub_index)) * 0.001;

    float imu_msg_north_velocity =
        (toValue<int16_t>(data, sub_index)) * velocity_coefficient;
    float imu_msg_east_velocity =
        (toValue<int16_t>(data, sub_index)) * velocity_coefficient;
    float imu_msg_ground_velocity =
        (toValue<int16_t>(data, sub_index)) * velocity_coefficient;
    uint8_t imu_msg_ins_status = toValue<uint8_t>(data, sub_index);
    sub_index = 44;
    uint8_t modestatus = toValue<uint8_t>(data, sub_index);
    sub_index = 46;
    int16_t data1 = toValue<uint16_t>(data, sub_index);
    int16_t data2 = toValue<uint16_t>(data, sub_index);
    int16_t data3 = toValue<uint16_t>(data, sub_index);

    sub_index = 52;
    uint32_t timiddle = toValue<uint32_t>(data, sub_index);
    double t_ms = timiddle * 2.5 * 0.0001;
    uint8_t loop_type = toValue<uint8_t>(data, sub_index);
    sub_index = 58;
    uint32_t t_week = (toValue<uint32_t>(data, sub_index));

    gtime_t ts_time = gpst2time_sdk(t_week, t_ms - 18);

    int64_t system_ms = (static_cast<double>(ts_time.time) * 1000.0) +
                        (static_cast<double>(ts_time.sec) * 1000.0);

    pos += m_lengthImu;

    // 俯仰角
    insdata.Pitch_deg = pitch;
    // 横滚角
    insdata.Roll_deg = roll;
    // 航向角
    insdata.Yaw_deg = yaw;

    insdata.GyroX = imu_msg_x_angular_velocity * deg_to_rad; /*gx*/
    insdata.GyroY = imu_msg_y_angular_velocity * deg_to_rad;
    insdata.GyroZ = imu_msg_z_angular_velocity * deg_to_rad;
    insdata.AccX_g = imu_msg_x_acc; /* 零偏修正后加速度计 */
    insdata.AccY_g = imu_msg_y_acc;
    insdata.AccZ_g = imu_msg_z_acc;

    // 这个是惯导融合后的输出
    insdata.Lon_deg = imu_msg_longitude;
    insdata.Lat_deg = imu_msg_latitude;
    insdata.Alt_m = imu_msg_altitude;

    insdata.VelN_mps =
        imu_msg_north_velocity; /*融合后 NED north velocity (m/s) */
    insdata.VelE_mps =
        imu_msg_east_velocity; /*融合后 NED east velocity (m/s) */
    insdata.VelD_mps =
        imu_msg_ground_velocity; /*融合后 NED down velocity (m/s) */
    insdata.InsStatus = imu_msg_ins_status;
    insdata.LoopType = loop_type;
    insdata.data1 = data1;
    insdata.data2 = data2;
    insdata.data3 = data3;
    insdata.ModeStatus = modestatus;
    insdata.SysTime_ms = system_ms;

    // Do not write csv.
    // std::vector<std::string> data_w;
    // data_w.push_back(std::to_string(pitch));
    // data_w.push_back(std::to_string(roll));
    // data_w.push_back(std::to_string(yaw));
    // data_w.push_back(std::to_string(imu_msg_x_angular_velocity));
    // data_w.push_back(std::to_string(imu_msg_y_angular_velocity));
    // data_w.push_back(std::to_string(imu_msg_z_angular_velocity));
    // data_w.push_back(std::to_string(imu_msg_x_acc));
    // data_w.push_back(std::to_string(imu_msg_y_acc));
    // data_w.push_back(std::to_string(imu_msg_z_acc));
    // data_w.push_back(std::to_string(imu_msg_longitude));
    // data_w.push_back(std::to_string(imu_msg_latitude));
    // data_w.push_back(std::to_string(imu_msg_altitude));
    // data_w.push_back(std::to_string(imu_msg_north_velocity));
    // data_w.push_back(std::to_string(imu_msg_east_velocity));
    // data_w.push_back(std::to_string(imu_msg_ground_velocity));
    // data_w.push_back(std::to_string(imu_msg_ins_status));
    // data_w.push_back(std::to_string(modestatus));
    // data_w.push_back(std::to_string(system_ms));

    // if (AppendCsv(filename0b, data_w)) {
    //   // std::cout << "数据成功写入到文件 " << filename << "\n";
    // } else {
    //   std::cerr << "写入文件时出现错误\n";
    // }

    update_ins = 1;
  } else {
    pos += 3;
  }
}

void Decode_0B::parseGnss(const uint8_t* data, int& pos) {
  int sub_index = 3;
  uint8_t check_sum = 0;
  int dataLength = getLength(m_typeGnss);
  /* check xor */
  for (int i = 0; i < dataLength - 1; ++i) {
    check_sum ^= data[i];
  }

  if (check_sum == data[dataLength - 1]) {
    double m_gnssMsg_longitude =
        (toValue<int32_t>(data, sub_index)) * 0.0000001;
    double m_gnssMsg_lon_sigma = (toValue<int16_t>(data, sub_index)) * 0.001;
    double m_gnssMsg_latitude = (toValue<int32_t>(data, sub_index)) * 0.0000001;
    double m_gnssMsg_lat_sigma = (toValue<int16_t>(data, sub_index)) * 0.001;
    double m_gnssMsg_altitude = (toValue<int32_t>(data, sub_index)) * 0.001;
    double m_gnssMsg_alt_sigma = (toValue<int16_t>(data, sub_index)) * 0.001;

    double m_gnssMsg_gps_fix = toValue<uint16_t>(data, sub_index);
    float m_gnssMsg_rtk_age = toValue<uint16_t>(data, sub_index);
    uint8_t m_gnssMsg_flags_pos = data[sub_index++];
    uint8_t m_gnssMsg_flags_vel = data[sub_index++];
    uint8_t m_gnssMsg_flags_attitude = data[sub_index++];
    uint8_t m_gnssMsg_flags_time = data[sub_index++];

    double m_gnssMsg_hor_vel = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_track_angle = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_ver_vel = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_latency_vel = (toValue<int16_t>(data, sub_index)) * 0.001;
    double m_gnssMsg_base_length = (toValue<int16_t>(data, sub_index)) * 0.001;

    double m_gnssMsg_yaw = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_yaw_sigma = (toValue<int16_t>(data, sub_index)) * 0.001;
    double m_gnssMsg_pitch = (toValue<int16_t>(data, sub_index)) * 0.001;
    double m_gnssMsg_pitch_sigma = (toValue<int16_t>(data, sub_index)) * 0.001;

    uint16_t utc_year_m = toValue<uint16_t>(data, sub_index);
    uint16_t utc_year = (utc_year_m & 0x3f) + 2000;
    // week_year = (((utc_year_m >> 6)&0x3ff) +2000);
    uint8_t utc_mon = data[sub_index++];
    uint8_t utc_day = data[sub_index++];
    uint8_t utc_hour = data[sub_index++];
    uint8_t utc_min = data[sub_index++];
    float utc_sec = toValue<uint16_t>(data, sub_index) * 0.001;

    double m_gnssMsg_ts_pos = toValue<uint32_t>(data, sub_index);
    double m_gnssMsg_ts_vel = toValue<uint32_t>(data, sub_index);
    double m_gnssMsg_ts_heading = toValue<uint32_t>(data, sub_index);
    double m_gnssMsg_state = data[sub_index++];
    double m_gnssMsg_num_master = data[sub_index++];
    sub_index++;

    double m_gnssMsg_gdop = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_pdop = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_hdop = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_htdop = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_tdop = (toValue<int16_t>(data, sub_index)) * 0.01;
    double m_gnssMsg_num_reserve = data[sub_index++];

    pos += m_lengthGnss;

    insdata.flag_pos = m_gnssMsg_flags_pos;
    // 这是融合rtk的输出
    insdata.Lon_gnss_deg = m_gnssMsg_longitude;
    insdata.Lat_gnss_deg = m_gnssMsg_latitude;
    insdata.Alt_gnss_m = m_gnssMsg_altitude;
    insdata.differential_age = m_gnssMsg_rtk_age;

    // Do not write csv.
    // std::vector<std::string> data_w;
    // data_w.push_back(std::to_string(m_gnssMsg_longitude));
    // data_w.push_back(std::to_string(m_gnssMsg_lon_sigma));
    // data_w.push_back(std::to_string(m_gnssMsg_latitude));
    // data_w.push_back(std::to_string(m_gnssMsg_lat_sigma));
    // data_w.push_back(std::to_string(m_gnssMsg_altitude));
    // data_w.push_back(std::to_string(m_gnssMsg_alt_sigma));
    // data_w.push_back(std::to_string(m_gnssMsg_gps_fix));
    // data_w.push_back(std::to_string(m_gnssMsg_rtk_age));
    // // data_w.push_back(std::to_string(m_gnssMsg_msg_type));
    // // data_w.push_back(std::to_string(m_gnssMsg_flags_pos));
    // // data_w.push_back(std::to_string(m_gnssMsg_flags_vel));
    // // data_w.push_back(std::to_string(m_gnssMsg_flags_attitude));
    // // data_w.push_back(std::to_string(m_gnssMsg_flags_time));
    // // data_w.push_back(std::to_string(m_gnssMsg_hor_vel));
    // data_w.push_back(std::to_string(m_gnssMsg_track_angle));
    // data_w.push_back(std::to_string(m_gnssMsg_ver_vel));
    // data_w.push_back(std::to_string(m_gnssMsg_latency_vel));
    // data_w.push_back(std::to_string(m_gnssMsg_base_length));
    // data_w.push_back(std::to_string(m_gnssMsg_yaw));
    // data_w.push_back(std::to_string(m_gnssMsg_yaw_sigma));
    // data_w.push_back(std::to_string(m_gnssMsg_pitch));
    // data_w.push_back(std::to_string(m_gnssMsg_pitch_sigma));
    // data_w.push_back(std::to_string(utc_year));
    // data_w.push_back(std::to_string(utc_mon));
    // data_w.push_back(std::to_string(utc_day));
    // data_w.push_back(std::to_string(utc_hour));
    // data_w.push_back(std::to_string(utc_min));
    // data_w.push_back(std::to_string(utc_sec));
    // data_w.push_back(std::to_string(m_gnssMsg_ts_pos));
    // data_w.push_back(std::to_string(m_gnssMsg_ts_vel));
    // data_w.push_back(std::to_string(m_gnssMsg_ts_heading));
    // data_w.push_back(std::to_string(m_gnssMsg_state));
    // data_w.push_back(std::to_string(m_gnssMsg_num_master));
    // data_w.push_back(std::to_string(m_gnssMsg_gdop));
    // data_w.push_back(std::to_string(m_gnssMsg_pdop));
    // data_w.push_back(std::to_string(m_gnssMsg_hdop));
    // // data_w.push_back(std::to_string(m_gnssMsg_vdop));
    // data_w.push_back(std::to_string(m_gnssMsg_tdop));
    // data_w.push_back(std::to_string(m_gnssMsg_num_reserve));

    // if (AppendCsv(filename2, data_w)) {
    //   // std::cout << "数据成功写入到文件 " << filename << "\n";
    // } else {
    //   std::cerr << "写入文件时出现错误\n";
    // }

  } else {
    pos += 3;
  }
}
