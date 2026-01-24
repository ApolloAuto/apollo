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
#pragma once

#include <cstdint>
#include <string>

#include "modules/common_msgs/sensor_msgs/gnss.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

constexpr uint32_t Message101B_ID = 0x553ACE02;

constexpr char GIAVP_MESSAGE_ID[] = "$GIAVP";
constexpr uint32_t GIAVP_MESSAGE_SIZE = 24;
constexpr char GPINS_MESSAGE_ID[] = "$GPINS";
constexpr uint32_t GPINS_MESSAGE_SIZE = 27;
constexpr char BESTPOSA_MESSAGE_ID[] = "#BESTPOSA";
constexpr uint32_t BESTPOSA_MESSAGE_SIZE = 30;
constexpr char HEADINGA_MESSAGE_ID[] = "#HEADINGA";
constexpr uint32_t HEADINGA_MESSAGE_SIZE = 26;

struct BraodGnss101BMessage {
  uint16_t gps_week;
  double gps_time;
  float heading;
  float pitch;
  float roll;
  double latitude;
  double longitude;
  float altitude;
  float ve;
  float vn;
  float vu;
  float baseline;
  uint8_t nsv1;
  uint8_t nsv2;
  uint8_t gps_status;
  uint8_t heading_status;
  uint8_t sys_status;
  uint8_t vehicle_align;
  uint8_t rtcm_status;
  uint8_t reserved;
  uint8_t speed_status;
  float vehicle_speed;
  float acc_x;
  float acc_y;
  float acc_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  uint16_t check_sum;
};

struct BroadGnssMessage {
  std::string message_type;
  double gps_timestamp_sec = 0;
  float heading;
  float gps_heading = -1.0;
  float pitch;
  float gps_pitch;
  double roll;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  double acc_x;
  double acc_y;
  double acc_z;
  double best_latitude = -1.0;
  double best_longitude = -1.0;
  float best_altitude = -1.0;
  float undulation = 10.0;
  double latitude;
  double longitude;
  double altitude;
  double ve;
  double vn;
  double vu;
  double lat_std = 0.0;
  double lon_std = 0.0;
  double alti_std = 0.0;
  float differential_age = 0.0;
  float solution_age = 0.0;
  double vn_std = 0.0;
  double ve_std = 0.0;
  double vu_std = 0.0;
  double roll_std = 0.0;
  double pitch_std = 0.0;
  double yaw_std = 0.0;
  uint8_t satellites_num = 0;
  double age;
  SolutionStatus solution_status;
  SolutionType solution_type;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
