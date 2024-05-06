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

// This defines an stream interface for communication via USB, Ethernet, etc.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "modules/common_msgs/drivers_msgs/can_card_parameter.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/gnss/util/macros.h"

namespace apollo {
namespace drivers {
namespace gnss {

// An abstract class of Stream.
// One should use the create_xxx() functions to create a Stream object.
class Stream {
 public:
  // Return a pointer to a Stream object. The caller should take ownership.
  static Stream *create_tcp(const char *address, uint16_t port,
                            uint32_t timeout_usec = 1000000);

  static Stream *create_udp(const char *address, uint16_t port,
                            uint32_t timeout_usec = 1000000);

  // Currently the following baud rates are supported:
  //  9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600.
  static Stream *create_serial(const char *device_name, uint32_t baud_rate,
                               uint32_t timeout_usec = 0);

  static Stream *create_ntrip(const std::string &address, uint16_t port,
                              const std::string &mountpoint,
                              const std::string &user,
                              const std::string &passwd,
                              uint32_t timeout_s = 30);
  static Stream *create_can(
      const apollo::drivers::canbus::CANCardParameter &parameter);

  virtual ~Stream() {}

  // Stream status.
  enum class Status {
    DISCONNECTED,
    CONNECTED,
    ERROR,
  };

  static constexpr size_t NUM_STATUS =
      static_cast<int>(Stream::Status::ERROR) + 1;
  Status get_status() const { return status_; }

  // Returns whether it was successful to connect.
  virtual bool Connect() = 0;

  // Returns whether it was successful to disconnect.
  virtual bool Disconnect() = 0;

  void RegisterLoginData(const std::vector<std::string> login_data) {
    login_data_.assign(login_data.begin(), login_data.end());
  }

  void Login() {
    for (size_t i = 0; i < login_data_.size(); ++i) {
      write(login_data_[i]);
      AINFO << "Login: " << login_data_[i];
      // sleep a little to avoid overrun of the slow serial interface.
      cyber::Duration(0.5).Sleep();
    }
  }

  // Reads up to max_length bytes. Returns actually number of bytes read.
  virtual size_t read(uint8_t *buffer, size_t max_length) = 0;

  // Returns how many bytes it was successful to write.
  virtual size_t write(const uint8_t *buffer, size_t length) = 0;

  size_t write(const std::string &buffer) {
    return write(reinterpret_cast<const uint8_t *>(buffer.data()),
                 buffer.size());
  }

 protected:
  Stream() {}

  Status status_ = Status::DISCONNECTED;

 private:
  std::vector<std::string> login_data_;
  DISABLE_COPY_AND_ASSIGN(Stream);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
