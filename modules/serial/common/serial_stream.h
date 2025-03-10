/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 * Modified by WheelOS, 2025. All Rights Reserved.
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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <thread>

#include <linux/netlink.h>
#include <linux/serial.h>

#include "cyber/cyber.h"

namespace apollo {
namespace serial {

class SerialStream {
 public:
  SerialStream(const char* device_name, speed_t baud_rate,
               uint32_t timeout_usec);
  ~SerialStream();

  virtual bool Connect();
  virtual bool Disconnect();
  virtual size_t read(uint8_t* buffer, size_t max_length);
  virtual size_t write(const uint8_t* data, size_t length);

 private:
  SerialStream() {}
  void open();
  void close();
  bool configure_port(int fd);
  bool wait_readable(uint32_t timeout_us);
  bool wait_writable(uint32_t timeout_us);
  void check_remove();

  std::string device_name_;
  speed_t baud_rate_;
  uint32_t bytesize_;
  uint32_t parity_;
  uint32_t stopbits_;
  uint32_t flowcontrol_;
  uint32_t byte_time_us_;

  uint32_t timeout_usec_;
  int fd_;
  int errno_;
  bool is_open_;
};

}  // namespace serial
}  // namespace apollo
