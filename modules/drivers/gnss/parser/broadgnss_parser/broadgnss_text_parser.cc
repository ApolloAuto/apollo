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
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/broadgnss_parser/broadgnss_base_parser.h"
#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

class BroadGnssTextParser : public BroadGnssBaseParser {
 public:
  explicit BroadGnssTextParser(const config::Config &config)
      : BroadGnssBaseParser(config) {}
  bool PrepareMessage(const uint8_t &channel) override;

 private:
  void PrepareMessageCommon(const std::vector<std::string> &fields);
  void PrepareMessageGIAVP(const std::vector<std::string> &fields);
  void PrepareMessageGPINS(const std::vector<std::string> &fields);
  void PrepareMessageBESTPOSA(const std::vector<std::string> &fields);
  void PrepareMessageHEADINGA(const std::vector<std::string> &fields);

  std::string input_str_;
  ASCIIParser ascii_parser_;
};

Parser *Parser::CreateBroadGnssText(const config::Config &config) {
  return new BroadGnssTextParser(config);
}

bool BroadGnssTextParser::PrepareMessage(const uint8_t &channel) {
  ADEBUG << "BroadGnss ASCII is: " << data_;

  ascii_parser_.Add(data_, data_end_, channel);
  auto &message = ascii_parser_.GetMessage();
  if (message.empty()) {
    return false;
  }

  std::stringstream ss(message.substr(0, message.rfind('*')));
  std::vector<std::string> fields;
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
                     std::string::npos ||
                 fields[i].find_first_of("0123456789") == std::string::npos) {
        AERROR << "BroadGnss ASCII message field error: " << fields[i]
               << ", input str: " << message;
        return false;
      }
    }
    return true;
  };
  broadgnss_message_.message_type = fields[0];
  if (fields[0] == GIAVP_MESSAGE_ID) {
    if (fields.size() != GIAVP_MESSAGE_SIZE) {
      AERROR << "BroadGnss GIAVP message format error: " << message;
      return false;
    }
    if (!valid_fields()) {
      return false;
    }
    PrepareMessageGIAVP(fields);
    return true;
  } else if (fields[0] == GPINS_MESSAGE_ID) {
    if (fields.size() != GPINS_MESSAGE_SIZE) {
      AERROR << "BroadGnss GPINS message format error:" << message;
      return false;
    }
    if (!valid_fields()) {
      return false;
    }
    PrepareMessageGPINS(fields);
    return true;
  } else if (fields[0] == BESTPOSA_MESSAGE_ID) {
    if (fields.size() != BESTPOSA_MESSAGE_SIZE) {
      AERROR << "BroadGnss BESTPOSA message format error: " << message;
      return false;
    }
    PrepareMessageBESTPOSA(fields);
    return true;
  } else if (fields[0] == HEADINGA_MESSAGE_ID) {
    if (fields.size() != HEADINGA_MESSAGE_SIZE) {
      AERROR << "BroadGnss HEADINGA message format error: " << message;
      return false;
    }
    PrepareMessageHEADINGA(fields);
    return true;
  }
  return false;
}

void BroadGnssTextParser::PrepareMessageCommon(
    const std::vector<std::string> &fields) {
  broadgnss_message_.gps_timestamp_sec =
      std::stoi(fields[1]) * SECONDS_PER_WEEK + std::stod(fields[2]);
  broadgnss_message_.heading = std::stod(fields[3]);
  broadgnss_message_.pitch = std::stod(fields[4]);
  broadgnss_message_.roll = std::stod(fields[5]);
  broadgnss_message_.latitude = std::stod(fields[6]);
  broadgnss_message_.longitude = std::stod(fields[7]);
  broadgnss_message_.altitude = std::stod(fields[8]);
  broadgnss_message_.ve = std::stod(fields[9]);
  broadgnss_message_.vn = std::stod(fields[10]);
  broadgnss_message_.vu = std::stod(fields[11]);
  broadgnss_message_.satellites_num = std::stoi(fields[13]);
}

void BroadGnssTextParser::PrepareMessageGIAVP(
    const std::vector<std::string> &fields) {
  PrepareMessageCommon(fields);
  int solution_type = std::stoi(fields[15]);
  int solution_status = 0;
  if (solution_type >= 4) {
    solution_status = 4;
  }
  PrepareMessageStatus(solution_status, solution_type);
  broadgnss_message_.acc_x = std::stod(fields[18]);
  broadgnss_message_.acc_y = std::stod(fields[19]);
  broadgnss_message_.acc_z = std::stod(fields[20]);
  broadgnss_message_.gyro_x = std::stod(fields[21]) * DEG_TO_RAD;
  broadgnss_message_.gyro_y = std::stod(fields[22]) * DEG_TO_RAD;
  broadgnss_message_.gyro_z = std::stod(fields[23]) * DEG_TO_RAD;
}

void BroadGnssTextParser::PrepareMessageGPINS(
    const std::vector<std::string> &fields) {
  PrepareMessageCommon(fields);
  PrepareMessageStatus(std::stoi(fields[17]), std::stoi(fields[15]));
  broadgnss_message_.acc_x = std::stod(fields[21]);
  broadgnss_message_.acc_y = std::stod(fields[22]);
  broadgnss_message_.acc_z = std::stod(fields[23]);
  broadgnss_message_.gyro_x = std::stod(fields[24]) * DEG_TO_RAD;
  broadgnss_message_.gyro_y = std::stod(fields[25]) * DEG_TO_RAD;
  broadgnss_message_.gyro_z = std::stod(fields[26]) * DEG_TO_RAD;
}

void BroadGnssTextParser::PrepareMessageBESTPOSA(
    const std::vector<std::string> &fields) {
  broadgnss_message_.best_latitude = std::stod(fields[11]);
  broadgnss_message_.best_longitude = std::stod(fields[12]);
  broadgnss_message_.best_altitude = std::stof(fields[13]);
  broadgnss_message_.undulation = std::stof(fields[14]);
  broadgnss_message_.lat_std = std::stof(fields[16]);
  broadgnss_message_.lon_std = std::stof(fields[17]);
  broadgnss_message_.alti_std = std::stof(fields[18]);
  broadgnss_message_.differential_age = std::stof(fields[20]);
  broadgnss_message_.solution_age = std::stof(fields[21]);
}

void BroadGnssTextParser::PrepareMessageHEADINGA(
    const std::vector<std::string> &fields) {
  broadgnss_message_.gps_heading = std::stof(fields[12]);
  broadgnss_message_.gps_pitch = std::stof(fields[13]);
  broadgnss_message_.yaw_std = std::stof(fields[15]);
  broadgnss_message_.pitch_std = std::stof(fields[16]);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
