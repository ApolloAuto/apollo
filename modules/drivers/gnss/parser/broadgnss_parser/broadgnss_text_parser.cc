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
  bool PrepareMessage() override;

 private:
  void PrepareMessageCommon(const std::vector<std::string> &fields);
  void PrepareMessageGIAVP(const std::vector<std::string> &fields);
  void PrepareMessageGPINS(const std::vector<std::string> &fields);

  BroadGnssTextProtocol protocol_;
  std::string input_str_;
};

Parser *Parser::CreateBroadGnssText(const config::Config &config) {
  return new BroadGnssTextParser(config);
}

bool BroadGnssTextParser::PrepareMessage() {
  ADEBUG << "BroadGnss ASCII is: " << data_;

  // message may be truncated, just save the last message
  const uint8_t *data_start = data_end_;
  for (; data_start != data_; --data_start) {
    if (*data_start == '$') {
      break;
    }
  }
  if (data_start != data_) {
    AWARN << "BroadGnss message has been truncated: " << data_;
  }

  if (*data_start != '$') {
    input_str_.append(reinterpret_cast<const char *>(data_start),
                      std::distance(data_start, data_end_));
  } else {
    input_str_.assign(reinterpret_cast<const char *>(data_start),
                      std::distance(data_start, data_end_));
  }

  if (*(data_end_ - 1) != 0x0A) {
    AWARN << "BroadGnss ASCII message is not complete: " << data_start;
    return false;
  } else {
    ADEBUG << "BroadGnss ASCII message is complete";
  }

  // find cs pos
  std::stringstream ss(input_str_.substr(0, input_str_.rfind('*')));
  std::vector<std::string> fields;
  for (std::string field; std::getline(ss, field, ',');) {
    fields.push_back(field);
  }
  if (fields.empty()) {
    return false;
  }
  if (fields[0] == protocol_.GIAVP) {
    if (fields.size() != protocol_.GIAVP_SIZE) {
      AERROR << "BroadGnss GIAVP message format error: " << data_;
      return false;
    }
    PrepareMessageGIAVP(fields);
    return true;
  } else if (fields[0] == protocol_.GPINS) {
    if (fields.size() != protocol_.GPINS_SIZE) {
      AERROR << "BroadGnss GPINS message format error:" << data_;
      return false;
    }
    PrepareMessageGPINS(fields);
    return true;
  }
  return false;
}

void BroadGnssTextParser::PrepareMessageCommon(
    const std::vector<std::string> &fields) {
  broadgnss_message_.gps_timestamp_sec =
      std::stoi(fields[1]) * SECONDS_PER_WEEK + std::stod(fields[2]);
  broadgnss_message_.unix_timestamp_sec =
      apollo::drivers::util::gps2unix(broadgnss_message_.gps_timestamp_sec);
  broadgnss_message_.heading = std::stod(fields[3]);
  broadgnss_message_.pitch = std::stod(fields[4]);
  broadgnss_message_.roll = std::stod(fields[5]);
  broadgnss_message_.latitude = std::stod(fields[6].empty() ? "0" : fields[6]);
  broadgnss_message_.longitude = std::stod(fields[7].empty() ? "0" : fields[7]);
  broadgnss_message_.altitude = std::stod(fields[8].empty() ? "0" : fields[8]);
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

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
