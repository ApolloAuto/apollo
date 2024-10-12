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
#include "modules/drivers/gnss/parser/forsense_parser/forsense_base_parser.h"
#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

struct ForsenseProtocol {
  std::string GPYJ = "$GPYJ";
  size_t GPYJ_SIZE = 24;
};

class ForsenseTextParser : public ForsenseBaseParser {
 public:
  explicit ForsenseTextParser(const config::Config &config)
      : ForsenseBaseParser(config) {}
  bool PrepareMessage() override;

 private:
  void PrepareMessageGPYJ(const std::vector<std::string> &fields);

  ForsenseProtocol protocol_;
  std::string input_str_;
};

Parser *Parser::CreateForsenseText(const config::Config &config) {
  return new ForsenseTextParser(config);
}

bool ForsenseTextParser::PrepareMessage() {
  ADEBUG << "Forsense ASCII is: " << data_;

  // message may be truncated, just save the last message
  const uint8_t *data_start = data_end_;
  for (; data_start != data_; --data_start) {
    if (*data_start == '$') {
      break;
    }
  }
  if (data_start != data_) {
    AWARN << "Forsense message has been truncated: " << data_;
  }

  if (*data_start != '$') {
    input_str_.append(reinterpret_cast<const char *>(data_start),
                      std::distance(data_start, data_end_));
  } else {
    input_str_.assign(reinterpret_cast<const char *>(data_start),
                      std::distance(data_start, data_end_));
  }

  if (*(data_end_ - 1) != 0x0A) {
    AWARN << "Forsense ASCII message is not complete: " << data_start;
    return false;
  }

  std::vector<std::string> fields;
  std::stringstream ss(input_str_);
  for (std::string field; std::getline(ss, field, ',');) {
    fields.push_back(field);
  }
  if (fields.empty()) {
    return false;
  }
  if (fields[0] == protocol_.GPYJ) {
    if (fields.size() < protocol_.GPYJ_SIZE) {
      AERROR << "GPYJ message format error: " << data_start;
      return false;
    }
    PrepareMessageGPYJ(fields);
    return true;
  } 
  return false;
}

void ForsenseTextParser::PrepareMessageGPYJ(
    const std::vector<std::string> &fields) {
  decode_message_.messageID = fields[0];
  decode_message_.GPSWeek = std::stoi(fields[1]);
  decode_message_.GPSTime = std::stod(fields[2]);
  decode_message_.gps_timestamp_sec =
      decode_message_.GPSWeek * SECONDS_PER_WEEK + decode_message_.GPSTime;
  decode_message_.Heading = std::stod(fields[3]);
  decode_message_.Pitch = std::stod(fields[4]);
  decode_message_.Roll = std::stod(fields[5]);
  decode_message_.GyroX = std::stod(fields[6]);
  decode_message_.GyroY = std::stod(fields[7]);
  decode_message_.GyroZ = std::stod(fields[8]);
  decode_message_.AccX = std::stod(fields[9]);
  decode_message_.AccY = std::stod(fields[10]);
  decode_message_.AccZ = std::stod(fields[11]);
  decode_message_.Latitude = std::stod(fields[12].empty() ? "0" : fields[12]);
  decode_message_.Longitude = std::stod(fields[13].empty() ? "0" : fields[13]);
  decode_message_.Altitude = std::stod(fields[14].empty() ? "0" : fields[14]);
  decode_message_.Ve = std::stod(fields[15]);
  decode_message_.Vn = std::stod(fields[16]);
  decode_message_.Vu = std::stod(fields[17]);
  decode_message_.V = std::stod(fields[18]);
  decode_message_.NSV1 = std::stoi(fields[19]);
  decode_message_.NSV2 = std::stoi(fields[20]);
  int status = std::stoi(fields[21]);
  PrepareMessageStatus(status % 10, status / 10);
  decode_message_.Age = std::stod(fields[22]);
  decode_message_.WarningCs = fields[23];
}



}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
