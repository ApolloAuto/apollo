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

#ifndef MODULES_DRIVERS_GNSS_PARSER_H_
#define MODULES_DRIVERS_GNSS_PARSER_H_

#include <stdint.h>

#include <google/protobuf/message.h>

#include "util/macros.h"

namespace apollo {
namespace drivers {
namespace gnss {

// convert gps time (base on Jan 6 1980) to system time (base on Jan 1 1970)
// notice: Jan 6 1980
//
// linux shell:
// time1 = date +%s -d"Jan 6, 1980 00:00:01"
// time2 = date +%s -d"Jan 1, 1970 00:00:01"
// dif_tick = time1-time2
// 315964800 = 315993601 - 28801

#define EPOCH_AND_SYSTEM_DIFF_SECONDS 315964800

// A general pointer to a protobuf message.
using MessagePtr = ::google::protobuf::Message *;

// A helper function that returns a pointer to a protobuf message of type T.
template <class T>
inline T *As(MessagePtr message_ptr) {
  return dynamic_cast<T *>(message_ptr);
}

// An abstract class of Parser.
// One should use the create_xxx() functions to create a Parser object.
class Parser {
 public:
  // Return a pointer to a NovAtel parser. The caller should take ownership.
  static Parser *create_novatel();

  // Return a pointer to a u-blox parser. The caller should take ownership.
  static Parser *create_ublox();

  virtual ~Parser() {}

  // Updates the parser with new data. The caller must keep the data valid until
  // get_message()
  // returns NONE.
  void update(const uint8_t *data, size_t length) {
    _data = data;
    _data_end = data + length;
  }

  void update(const std::string &data) {
    update(reinterpret_cast<const uint8_t *>(data.data()), data.size());
  }

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
  };

  // Gets a parsed protobuf message. The caller must consume the message before
  // calling another
  // get_message() or update();
  virtual MessageType get_message(MessagePtr &message_ptr) = 0;

 protected:
  Parser() {}

  // Point to the beginning and end of data. Do not take ownership.
  const uint8_t *_data = nullptr;
  const uint8_t *_data_end = nullptr;

 private:
  DISABLE_COPY_AND_ASSIGN(Parser);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // MODULES_DRIVERS_GNSS_PARSER_H_
