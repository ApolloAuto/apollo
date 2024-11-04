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

// An parser for decoding binary messages from a NovAtel receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include "modules/drivers/gnss/parser/broadgnss_parser/broadgnss_base_parser.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

namespace apollo {
namespace drivers {
namespace gnss {

BroadGnssBaseParser::BroadGnssBaseParser(const config::Config& config) {}

void BroadGnssBaseParser::GetMessages(MessageInfoVec* messages) {
  if (data_ == nullptr) {
    return;
  }
  if (!PrepareMessage()) {
    return;
  }

  FillGnssBestpos();
  FillImu();
  FillHeading();
  FillIns();
  FillInsStat();

  if (bestpos_ratecontrol_.check()) {
    messages->push_back(MessageInfo{MessageType::BEST_GNSS_POS,
                                    reinterpret_cast<MessagePtr>(&bestpos_)});
  }
  messages->push_back(
      MessageInfo{MessageType::IMU, reinterpret_cast<MessagePtr>(&imu_)});
  messages->push_back(MessageInfo{MessageType::HEADING,
                                  reinterpret_cast<MessagePtr>(&heading_)});
  messages->push_back(
      MessageInfo{MessageType::INS, reinterpret_cast<MessagePtr>(&ins_)});
  messages->push_back(MessageInfo{MessageType::INS_STAT,
                                  reinterpret_cast<MessagePtr>(&ins_stat_)});
}

void BroadGnssBaseParser::PrepareMessageStatus(const uint8_t& solution_status,
                                               const uint8_t& solution_type) {
  switch (solution_status) {
    case 0:
      broadgnss_message_.solution_status = SolutionStatus::COLD_START;
      break;
    case 1:
    case 2:
    case 3:
      broadgnss_message_.solution_status = SolutionStatus::INSUFFICIENT_OBS;
      break;
    case 4:
      broadgnss_message_.solution_status = SolutionStatus::SOL_COMPUTED;
      break;
    default:
      broadgnss_message_.solution_status = SolutionStatus::INSUFFICIENT_OBS;
  }
  switch (solution_type) {
    case 0:
      broadgnss_message_.solution_type = SolutionType::NONE;
      break;
    case 1:
    case 2:
    case 3:
    case 6:
      broadgnss_message_.solution_type = SolutionType::SINGLE;
      break;
    case 4:
      broadgnss_message_.solution_type = SolutionType::INS_RTKFIXED;
      break;
    case 5:
      broadgnss_message_.solution_type = SolutionType::INS_RTKFLOAT;
      break;
    default:
      broadgnss_message_.solution_type = SolutionType::NONE;
  }
}

void BroadGnssBaseParser::FillGnssBestpos() {
  bestpos_.set_measurement_time(broadgnss_message_.gps_timestamp_sec);
  bestpos_.set_sol_status(broadgnss_message_.solution_status);
  bestpos_.set_sol_type(broadgnss_message_.solution_type);
  bestpos_.set_latitude(broadgnss_message_.latitude);
  bestpos_.set_longitude(broadgnss_message_.longitude);
  bestpos_.set_height_msl(broadgnss_message_.altitude);
  bestpos_.set_latitude_std_dev(broadgnss_message_.lat_std);
  bestpos_.set_longitude_std_dev(broadgnss_message_.lon_std);
  bestpos_.set_height_std_dev(broadgnss_message_.alti_std);
  bestpos_.set_num_sats_tracked(broadgnss_message_.satellites_num);
  bestpos_.set_num_sats_in_solution(broadgnss_message_.satellites_num);
  bestpos_.set_num_sats_l1(broadgnss_message_.satellites_num);
  bestpos_.set_num_sats_multi(broadgnss_message_.satellites_num);
}

void BroadGnssBaseParser::FillIns() {
  ins_.mutable_euler_angles()->set_x(broadgnss_message_.roll * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_y(-broadgnss_message_.pitch * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_z(
      azimuth_deg_to_yaw_rad(broadgnss_message_.heading));
  ins_.mutable_position()->set_lon(broadgnss_message_.longitude);
  ins_.mutable_position()->set_lat(broadgnss_message_.latitude);
  ins_.mutable_position()->set_height(broadgnss_message_.altitude);
  ins_.mutable_linear_velocity()->set_x(broadgnss_message_.ve);
  ins_.mutable_linear_velocity()->set_y(broadgnss_message_.vn);
  ins_.mutable_linear_velocity()->set_z(broadgnss_message_.vu);
  rfu_to_flu(broadgnss_message_.acc_x, broadgnss_message_.acc_y,
             broadgnss_message_.acc_z, ins_.mutable_linear_acceleration());
  rfu_to_flu(broadgnss_message_.gyro_x, broadgnss_message_.gyro_y,
             broadgnss_message_.gyro_z, ins_.mutable_angular_velocity());
  ins_.set_measurement_time(broadgnss_message_.gps_timestamp_sec);
  ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());

  switch (broadgnss_message_.solution_type) {
    case SolutionType::INS_RTKFIXED:
    case SolutionType::NARROW_INT:
    case SolutionType::INS_RTKFLOAT:
    case SolutionType::NARROW_FLOAT:
      ins_.set_type(Ins::GOOD);
      break;
    case SolutionType::SINGLE:
      ins_.set_type(Ins::CONVERGING);
      break;
    default:
      ins_.set_type(Ins::INVALID);
      break;
  }
}

void BroadGnssBaseParser::FillInsStat() {
  ins_stat_.set_ins_status(broadgnss_message_.solution_status);
  ins_stat_.set_pos_type(broadgnss_message_.solution_type);
}

void BroadGnssBaseParser::FillImu() {
  rfu_to_flu(broadgnss_message_.acc_x, broadgnss_message_.acc_y,
             broadgnss_message_.acc_z, imu_.mutable_linear_acceleration());
  rfu_to_flu(broadgnss_message_.gyro_x, broadgnss_message_.gyro_y,
             broadgnss_message_.gyro_z, imu_.mutable_angular_velocity());
  imu_.set_measurement_time(broadgnss_message_.gps_timestamp_sec);
}

void BroadGnssBaseParser::FillHeading() {
  heading_.set_solution_status(broadgnss_message_.solution_status);
  heading_.set_position_type(broadgnss_message_.solution_type);
  heading_.set_measurement_time(broadgnss_message_.gps_timestamp_sec);
  heading_.set_heading(broadgnss_message_.heading);
  heading_.set_pitch(broadgnss_message_.pitch);
  heading_.set_heading_std_dev(broadgnss_message_.yaw_std);
  heading_.set_pitch_std_dev(broadgnss_message_.pitch_std);
  // heading_.set_station_id("0");
  heading_.set_satellite_number_multi(broadgnss_message_.satellites_num);
  heading_.set_satellite_soulution_number(broadgnss_message_.satellites_num);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
