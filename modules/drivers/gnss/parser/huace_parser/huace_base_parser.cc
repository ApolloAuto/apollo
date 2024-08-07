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
#include "modules/drivers/gnss/parser/huace_parser/huace_base_parser.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

namespace apollo {
namespace drivers {
namespace gnss {

HuaCeBaseParser::HuaCeBaseParser(const config::Config& config) {}

void HuaCeBaseParser::GetMessages(MessageInfoVec* messages) {
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

void HuaCeBaseParser::PrepareMessageStatus(const uint8_t& system_state,
                                           const uint8_t& satellite_status) {
  switch (system_state) {
    case 0:
      decode_message_.solution_status = SolutionStatus::COLD_START;
      break;
    case 1:
    case 2:
      decode_message_.solution_status = SolutionStatus::SOL_COMPUTED;
      break;
    default:
      decode_message_.solution_status = SolutionStatus::INSUFFICIENT_OBS;
  }
  switch (satellite_status) {
    case 0:
      decode_message_.solution_type = SolutionType::NONE;
      break;
    case 1:
    case 6:
      decode_message_.solution_type = SolutionType::SINGLE;
      break;
    case 2:
    case 7:
      decode_message_.solution_type = SolutionType::PSRDIFF;
      break;
    case 3:
      decode_message_.solution_type = SolutionType::PROPOGATED;
      break;
    case 4:
      decode_message_.solution_type = SolutionType::INS_RTKFIXED;
      break;
    case 5:
      decode_message_.solution_type = SolutionType::INS_RTKFLOAT;
      break;
    case 8:
      decode_message_.solution_type = SolutionType::NARROW_INT;
      break;
    case 9:
      decode_message_.solution_type = SolutionType::NARROW_FLOAT;
      break;
    default:
      decode_message_.solution_type = SolutionType::NONE;
  }
}

void HuaCeBaseParser::FillGnssBestpos() {
  bestpos_.set_measurement_time(decode_message_.gps_timestamp_sec);
  bestpos_.set_sol_status(decode_message_.solution_status);
  bestpos_.set_sol_type(decode_message_.solution_type);
  bestpos_.set_latitude(decode_message_.Latitude);
  bestpos_.set_longitude(decode_message_.Longitude);
  bestpos_.set_height_msl(decode_message_.Altitude);
  bestpos_.set_latitude_std_dev(decode_message_.lat_std);
  bestpos_.set_longitude_std_dev(decode_message_.lon_std);
  bestpos_.set_height_std_dev(decode_message_.alti_std);
  bestpos_.set_num_sats_tracked(decode_message_.satellites_num);
  bestpos_.set_num_sats_in_solution(decode_message_.satellites_num);
  bestpos_.set_num_sats_l1(decode_message_.satellites_num);
  bestpos_.set_num_sats_multi(decode_message_.satellites_num);
}

void HuaCeBaseParser::FillIns() {
  ins_.mutable_euler_angles()->set_x(decode_message_.Roll * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_y(-decode_message_.Pitch * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_z(
      azimuth_deg_to_yaw_rad(decode_message_.Heading));
  ins_.mutable_position()->set_lon(decode_message_.Longitude);
  ins_.mutable_position()->set_lat(decode_message_.Latitude);
  ins_.mutable_position()->set_height(decode_message_.Altitude);
  ins_.mutable_linear_velocity()->set_x(decode_message_.Ve);
  ins_.mutable_linear_velocity()->set_y(decode_message_.Vn);
  ins_.mutable_linear_velocity()->set_z(decode_message_.Vu);
  rfu_to_flu(decode_message_.AccX, decode_message_.AccY, decode_message_.AccZ,
             ins_.mutable_linear_acceleration());
  rfu_to_flu(decode_message_.GyroX, decode_message_.GyroY,
             decode_message_.GyroZ, ins_.mutable_angular_velocity());
  ins_.set_measurement_time(decode_message_.gps_timestamp_sec);
  ins_.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());

  switch (decode_message_.solution_type) {
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

void HuaCeBaseParser::FillInsStat() {
  ins_stat_.set_ins_status(decode_message_.solution_status);
  ins_stat_.set_pos_type(decode_message_.solution_type);
}

void HuaCeBaseParser::FillImu() {
  rfu_to_flu(decode_message_.AccX, decode_message_.AccY, decode_message_.AccZ,
             imu_.mutable_linear_acceleration());
  rfu_to_flu(decode_message_.GyroX, decode_message_.GyroY,
             decode_message_.GyroZ, imu_.mutable_angular_velocity());
  imu_.set_measurement_time(decode_message_.gps_timestamp_sec);
}

void HuaCeBaseParser::FillHeading() {
  heading_.set_solution_status(decode_message_.solution_status);
  heading_.set_position_type(decode_message_.solution_type);
  heading_.set_measurement_time(decode_message_.gps_timestamp_sec);
  heading_.set_heading(decode_message_.Heading);
  heading_.set_pitch(decode_message_.Pitch);
  heading_.set_heading_std_dev(decode_message_.yaw_std);
  heading_.set_pitch_std_dev(decode_message_.pitch_std);
  // heading_.set_station_id("0");
  heading_.set_satellite_number_multi(decode_message_.satellites_num);
  heading_.set_satellite_soulution_number(decode_message_.satellites_num);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
