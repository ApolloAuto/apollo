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
#include "modules/drivers/gnss/parser/cxzl_parser/cxzl_base_parser.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

namespace apollo {
namespace drivers {
namespace gnss {

#define G_EARTH 9.806665
// #define DEG_TO_RAD 0.0174532925


CxzlBaseParser::CxzlBaseParser(const config::Config& config) 
{

}

void CxzlBaseParser::GetMessages(MessageInfoVec* messages) {
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

Parser *Parser::CreateCxzlCfg(const config::Config &config) {
  return new CxzlBaseParser(config);
}


void CxzlBaseParser::PrepareMessageStatus(const uint8_t& system_state,
                                           const uint8_t& satellite_status) {

}

void CxzlBaseParser::FillGnssBestpos() {
  bestpos_.set_measurement_time(decode_message_.gps_timestamp_sec);
  bestpos_.set_latitude(decode_message_.Latitude);
  bestpos_.set_longitude(decode_message_.Longitude);
  bestpos_.set_height_msl(decode_message_.Altitude);
  bestpos_.set_latitude_std_dev(decode_message_.Pe_std);
  bestpos_.set_longitude_std_dev(decode_message_.Pn_std);
  bestpos_.set_height_std_dev(decode_message_.Pu_std);
  bestpos_.set_num_sats_tracked(decode_message_.NSVD1);
  bestpos_.set_num_sats_in_solution(decode_message_.NSV1);
  bestpos_.set_num_sats_l1(decode_message_.NSV1);
  bestpos_.set_num_sats_multi(decode_message_.NSV1);
}

void CxzlBaseParser::FillIns() {
  ins_.mutable_euler_angles()->set_x(decode_message_.Roll * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_y(decode_message_.Pitch * DEG_TO_RAD);
  ins_.mutable_euler_angles()->set_z(decode_message_.Heading * DEG_TO_RAD);
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

  switch (decode_message_.Satellite_status) {
    case CxzlSatelliteStatusT::POS_LOCATEDIRECT_DIFF:
    case CxzlSatelliteStatusT::POS_LOCATEDIRECT_COMB:
    case CxzlSatelliteStatusT::POS_LOCATEDIRECT_FIXED:
    case CxzlSatelliteStatusT::POS_LOCATEDIRECT_FLOAT:
    case CxzlSatelliteStatusT::POS_LOCATE_DIFF:
    case CxzlSatelliteStatusT::POS_LOCATE_FIXED:
    case CxzlSatelliteStatusT::POS_LOCATE_FLOAT:
      ins_.set_type(Ins::GOOD);
      break;
    case CxzlSatelliteStatusT::POS_LOCATEDIRECT_SINGLE:
    case CxzlSatelliteStatusT::POS_LOCATE_SINGLE:
      ins_.set_type(Ins::CONVERGING);
      break;
    default:
      ins_.set_type(Ins::INVALID);
      break;
  }
}

void CxzlBaseParser::FillInsStat() {
  switch(decode_message_.Solution_status)
  {
    case CxzlSoluStatusT::INIT_STATUS:
      ins_stat_.set_ins_status(SolutionStatus::SOL_COMPUTED);
      bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
      heading_.set_solution_status(SolutionStatus::SOL_COMPUTED);
      break;
    case CxzlSoluStatusT::SATELLITE_NAVIGATION:
      ins_stat_.set_ins_status(SolutionStatus::SOL_COMPUTED);
      bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
      heading_.set_solution_status(SolutionStatus::SOL_COMPUTED);
      break;
    case CxzlSoluStatusT::INTEGRATED_NAVIGATION:
      ins_stat_.set_ins_status(SolutionStatus::SOL_COMPUTED);
      bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
      heading_.set_solution_status(SolutionStatus::SOL_COMPUTED);
      break;
    case CxzlSoluStatusT::INERTIAL_NAVIGATION:
      ins_stat_.set_ins_status(SolutionStatus::SOL_COMPUTED);
      bestpos_.set_sol_status(SolutionStatus::SOL_COMPUTED);
      heading_.set_solution_status(SolutionStatus::SOL_COMPUTED);
      break;
    default:
      break;
  }
  if ((decode_message_.Solution_status == INTEGRATED_NAVIGATION)
    || (decode_message_.Solution_status == INERTIAL_NAVIGATION))
  {
    switch(decode_message_.Satellite_status)
    {
      case CxzlSatelliteStatusT::POS_NONE:
        ins_stat_.set_pos_type(SolutionType::NONE);
        bestpos_.set_sol_type(SolutionType::NONE);
        heading_.set_position_type(SolutionType::NONE);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_SINGLE:
      case CxzlSatelliteStatusT::POS_LOCATE_SINGLE:
        ins_stat_.set_pos_type(SolutionType::SINGLE);
        bestpos_.set_sol_type(SolutionType::SINGLE);
        heading_.set_position_type(SolutionType::SINGLE);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_DIFF:
      case CxzlSatelliteStatusT::POS_LOCATE_DIFF:
        ins_stat_.set_pos_type(SolutionType::INS_PSRDIFF);
        bestpos_.set_sol_type(SolutionType::INS_PSRDIFF);
        heading_.set_position_type(SolutionType::INS_PSRDIFF);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_FIXED:
      case CxzlSatelliteStatusT::POS_LOCATE_FIXED:
        ins_stat_.set_pos_type(SolutionType::INS_RTKFIXED);
        bestpos_.set_sol_type(SolutionType::INS_RTKFIXED);
        heading_.set_position_type(SolutionType::INS_RTKFIXED);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_FLOAT:
      case CxzlSatelliteStatusT::POS_LOCATE_FLOAT:
        ins_stat_.set_pos_type(SolutionType::INS_RTKFLOAT);
        bestpos_.set_sol_type(SolutionType::INS_RTKFLOAT);
        heading_.set_position_type(SolutionType::INS_RTKFLOAT);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_COMB:
      default:
        ins_stat_.set_pos_type(SolutionType::NONE);
        bestpos_.set_sol_type(SolutionType::NONE);
        heading_.set_position_type(SolutionType::NONE);
        break;
    }
  }
  else
  {
    switch(decode_message_.Satellite_status)
    {
      case CxzlSatelliteStatusT::POS_NONE:
        ins_stat_.set_pos_type(SolutionType::NONE);
        bestpos_.set_sol_type(SolutionType::NONE);
        heading_.set_position_type(SolutionType::NONE);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_SINGLE:
      case CxzlSatelliteStatusT::POS_LOCATE_SINGLE:
        ins_stat_.set_pos_type(SolutionType::SINGLE);
        bestpos_.set_sol_type(SolutionType::SINGLE);
        heading_.set_position_type(SolutionType::SINGLE);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_DIFF:
      case CxzlSatelliteStatusT::POS_LOCATE_DIFF:
        ins_stat_.set_pos_type(SolutionType::PSRDIFF);
        bestpos_.set_sol_type(SolutionType::PSRDIFF);
        heading_.set_position_type(SolutionType::PSRDIFF);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_FIXED:
      case CxzlSatelliteStatusT::POS_LOCATE_FIXED:
        ins_stat_.set_pos_type(SolutionType::FIXEDPOS);
        bestpos_.set_sol_type(SolutionType::FIXEDPOS);
        heading_.set_position_type(SolutionType::FIXEDPOS);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_FLOAT:
      case CxzlSatelliteStatusT::POS_LOCATE_FLOAT:
        ins_stat_.set_pos_type(SolutionType::NARROW_FLOAT);
        bestpos_.set_sol_type(SolutionType::NARROW_FLOAT);
        heading_.set_position_type(SolutionType::NARROW_FLOAT);
        break;
      case CxzlSatelliteStatusT::POS_LOCATEDIRECT_COMB:
      default:
        ins_stat_.set_pos_type(SolutionType::NONE);
        bestpos_.set_sol_type(SolutionType::NONE);
        heading_.set_position_type(SolutionType::NONE);
        break;
    }
  }
}

void CxzlBaseParser::FillImu() {
  rfu_to_flu(decode_message_.AccX, decode_message_.AccY, decode_message_.AccZ,
             imu_.mutable_linear_acceleration());
  rfu_to_flu(decode_message_.GyroX, decode_message_.GyroY,
             decode_message_.GyroZ, imu_.mutable_angular_velocity());
  imu_.set_measurement_time(decode_message_.gps_timestamp_sec);
}

void CxzlBaseParser::FillHeading() {
  heading_.set_measurement_time(decode_message_.gps_timestamp_sec);
  heading_.set_heading(decode_message_.Heading);
  heading_.set_pitch(decode_message_.Pitch);
  heading_.set_heading_std_dev(decode_message_.heading_std);
  heading_.set_pitch_std_dev(decode_message_.pitch_std);
  // heading_.set_station_id("0");
  heading_.set_satellite_number_multi(decode_message_.NSV1);
  heading_.set_satellite_soulution_number(decode_message_.NSV1);
}


bool CxzlBaseParser::PrepareMessage() {
//   ADEBUG << "CXZL ASCII is: " << data_;

  // message may be truncated, just save the last message
  const uint8_t *data_start = data_;
  for (; data_start != data_; --data_start) {
    if (*data_start == '$')  {
      break;
    }
  }

  if (data_start != data_) {
    AWARN << "CXZL message has been truncated: " << data_;
  }

  if (*data_start != '$') {
    input_str.append(reinterpret_cast<const char *>(data_start),
                      std::distance(data_start, data_end_));
  } else {
    input_str.assign(reinterpret_cast<const char *>(data_start),
                      std::distance(data_start, data_end_));
  }

  if (*(data_end_ - 1) != 0x0A) {
    AWARN << "CXZL ASCII message is not complete: " << data_start;
    return false;
  }

  std::vector<std::string> fields;
  std::string checknum = input_str.substr(input_str.find('*')+1, input_str.find(0x0D));

  std::stringstream ss(input_str.substr(0, input_str.rfind('*')));
  for (std::string field; std::getline(ss, field, ',');) {
    fields.push_back(field);
  }
  if (fields.empty()) {
    return false;
  }
  auto valid_fields = [&]() -> bool {
    for (size_t i = 1; i < fields.size(); ++i) {
      if (fields[i].empty()) {
        fields[i] = "0";
      } else if (fields[i].find_first_not_of("0123456789.- ") !=
                 std::string::npos) {
        AERROR << "CXZL ASCII message field error: " << fields[i]
               << ", input str: " << input_str;
        return false;
      }
    }
    return true;
  };
  if (fields[0] == protocol_.cxhead) {
    if (fields.size() < protocol_.CXZL_SIZE) {
      AERROR << "CXINSPVA message format error: " << data_start;
      return false;
    }
    if (!valid_fields()) {
      return false;
    }
    PrepareMessageCxzl(fields);
    return true;
  }
  return false;
}

void CxzlBaseParser::PrepareMessageCxzl(const std::vector<std::string> &fields) {

  decode_message_.GPSWeek = std::stoi(fields[1]);
  decode_message_.GPSTime = std::stod(fields[2]);
  decode_message_.gps_timestamp_sec =
      decode_message_.GPSWeek * SECONDS_PER_WEEK + decode_message_.GPSTime;

  decode_message_.GyroX = std::stod(fields[3]) * DEG_TO_RAD;
  decode_message_.GyroY = std::stod(fields[4]) * DEG_TO_RAD;
  decode_message_.GyroZ = std::stod(fields[5]) * DEG_TO_RAD;
  decode_message_.AccX = std::stod(fields[6]) * G_EARTH;
  decode_message_.AccY = std::stod(fields[7]) * G_EARTH;
  decode_message_.AccZ = std::stod(fields[8]) * G_EARTH;

  int sys_status = std::stoi(fields[9]);
  switch(sys_status)
  {
    case 0:
      decode_message_.Solution_status = CxzlSoluStatusT::INIT_STATUS;
      break;
    case 1:
      decode_message_.Solution_status = CxzlSoluStatusT::SATELLITE_NAVIGATION;
      break;
    case 2:
      decode_message_.Solution_status = CxzlSoluStatusT::INTEGRATED_NAVIGATION;
      break;
    case 3:
      decode_message_.Solution_status = CxzlSoluStatusT::INERTIAL_NAVIGATION;
      break;
    default:
      break;
  }
  decode_message_.NSV1 = std::stoi(fields[10]);

  int satel_status = std::stoi(fields[11]);
  switch(satel_status)
  {
    case 0:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_NONE;
      break;
    case 1:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATEDIRECT_SINGLE;
      break;
    case 2:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATEDIRECT_DIFF;
      break;
    case 3:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATEDIRECT_COMB;
      break;
    case 4:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATEDIRECT_FIXED;
      break;
    case 5:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATEDIRECT_FLOAT;
      break;
    case 6:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATE_SINGLE;
      break;
    case 7:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATE_DIFF;
      break;
    case 8:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATE_FIXED;
      break;
    case 9:
      decode_message_.Satellite_status = CxzlSatelliteStatusT::POS_LOCATE_FLOAT;
      break;
    default:
      break;
  }
  decode_message_.NSV2 = std::stoi(fields[12]);
  decode_message_.Age = std::stod(fields[13]);
  decode_message_.NSVD1 = std::stoi(fields[14]);
  decode_message_.NSVD2 = std::stoi(fields[15]);

  decode_message_.Altitude = std::stod(fields[16]);
  decode_message_.Pe_std = std::stod(fields[17]);
  decode_message_.Pn_std = std::stod(fields[18]);
  decode_message_.Pu_std = std::stod(fields[19]);
  decode_message_.Ve = std::stod(fields[20]);
  decode_message_.Vn = std::stod(fields[21]);
  decode_message_.Vu = std::stod(fields[22]);
  decode_message_.V = std::stod(fields[23]);

  decode_message_.vn_std = std::stod(fields[24]);
  decode_message_.ve_std = std::stod(fields[25]);
  decode_message_.ve_std = std::stod(fields[26]);
  decode_message_.v_std = std::stod(fields[27]);
  decode_message_.Veh_AccX = std::stod(fields[28]);
  decode_message_.Veh_AccY = std::stod(fields[29]);
  decode_message_.Veh_AccZ = std::stod(fields[30]);

  decode_message_.Roll = std::stod(fields[31]);
  decode_message_.Pitch = std::stod(fields[32]);
  decode_message_.Heading = std::stod(fields[33]);
  decode_message_.roll_std = std::stod(fields[34]);
  decode_message_.pitch_std = std::stod(fields[35]);
  decode_message_.heading_std = std::stod(fields[36]);
  decode_message_.Veh_GyroX = std::stod(fields[37]);
  decode_message_.Veh_GyroY = std::stod(fields[38]);
  decode_message_.Veh_GyroZ = std::stod(fields[39]);

  decode_message_.Longitude = std::stod(fields[40]);
  decode_message_.Latitude = std::stod(fields[41]);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
