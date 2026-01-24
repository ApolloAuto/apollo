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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "google/protobuf/message.h"

#include "modules/drivers/gnss/proto/config.pb.h"

#include "modules/drivers/gnss/util/macros.h"

namespace apollo {
namespace drivers {
namespace gnss {

// A general pointer to a protobuf message.
using MessagePtr = ::google::protobuf::Message *;
enum class MessageType {
  NONE,
  GNSS,
  GNSS_RANGE,
  IMU,
  INS,
  INS_STAT,
  WHEEL,
  EPHEMERIDES,
  OBSERVATION,
  GPGGA,
  BDSEPHEMERIDES,
  RAWIMU,
  GPSEPHEMERIDES,
  GLOEPHEMERIDES,
  BEST_GNSS_POS,
  HEADING,
};
struct MessageInfo {
  MessageType type;
  MessagePtr message_ptr;
};
using MessageInfoVec = std::vector<MessageInfo>;

// convert gps time (base on Jan 6 1980) to system time (base on Jan 1 1970)
// notice: Jan 6 1980
//
// linux shell:
// time1 = date +%s -d"Jan 6, 1980 00:00:01"
// time2 = date +%s -d"Jan 1, 1970 00:00:01"
// dif_tick = time1-time2
// 315964800 = 315993601 - 28801

#define EPOCH_AND_SYSTEM_DIFF_SECONDS 315964800

// A helper function that returns a pointer to a protobuf message of type T.
template <class T>
inline T *As(::google::protobuf::Message *message_ptr) {
  return dynamic_cast<T *>(message_ptr);
}

// An abstract class of Parser.
// One should use the create_xxx() functions to create a Parser object.
class Parser {
 public:
  // Return a pointer to a parser. The caller should take ownership.
  static Parser *CreateNovatel(const config::Config &config);
  static Parser *CreateHuaCeText(const config::Config &config);
  static Parser *CreateAsensingBinary(const config::Config &config);
  static Parser *CreateAsensingCan(const config::Config &config);
  static Parser *CreateBroadGnssText(const config::Config &config);

  static Parser *CreateParser(const config::Config &config) {
    switch (config.data().format()) {
      case config::Stream::NOVATEL_BINARY:
        return Parser::CreateNovatel(config);
      case config::Stream::HUACE_TEXT:
        return Parser::CreateHuaCeText(config);
      case config::Stream::ASENSING_BINARY:
        return Parser::CreateAsensingBinary(config);
      case config::Stream::ASENSING_CAN:
        return Parser::CreateAsensingCan(config);
      case config::Stream::BROADGNSS_TEXT:
        return Parser::CreateBroadGnssText(config);
      default:
        return nullptr;
    }
  }

  // Return a pointer to rtcm v3 parser. The caller should take ownership.
  static Parser *CreateRtcmV3(bool is_base_station = false);

  virtual ~Parser() {}

  // Updates the parser with new data. The caller must keep the data valid until
  // GetMessage()
  // returns NONE.
  void Update(const uint8_t *data, size_t length) {
    data_ = data;
    data_end_ = data + length;
  }

  void Update(const std::string &data) {
    Update(reinterpret_cast<const uint8_t *>(data.data()), data.size());
  }
  // Gets a parsed protobuf message. The caller must consume the message before
  // calling another
  // GetMessage() or Update();
  virtual MessageType GetMessage(MessagePtr *message_ptr) {
    return MessageType::NONE;
  }

  void GetMessagesByChannel(const uint8_t &channel, MessageInfoVec *messages) {
    if (channel == 0) {
      GetMessages(messages);
    } else {
      GetMessages(channel, messages);
    }
  }

  virtual void GetMessages(const uint8_t &channel, MessageInfoVec *messages) {}

  virtual void GetMessages(MessageInfoVec *messages) {}

  virtual bool GetInsStat(MessagePtr *message_ptr) { return false; }

 protected:
  Parser() {}

  // Point to the beginning and end of data. Do not take ownership.
  const uint8_t *data_ = nullptr;
  const uint8_t *data_end_ = nullptr;

 private:
  DISABLE_COPY_AND_ASSIGN(Parser);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
