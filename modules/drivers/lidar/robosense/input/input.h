/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include <netdb.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
namespace apollo {
namespace drivers {
namespace robosense {
const int RSLIDAR_PKT_LEN = 1248;
enum InputState {
  INPUT_OK = 0,
  INPUT_TIMEOUT = 1,
  INPUT_ERROR = 2,
  INPUT_DIFOP = 4,
  INPUT_MSOP = 8,
  INPUT_EXIT = 16
};

class Input {
 public:
  Input(const uint16_t &msop_port, const uint16_t &difop_port);
  ~Input();
  InputState getPacket(uint8_t *pkt, uint32_t timeout);

 private:
  int setUpSocket(uint16_t port);
  int msop_fd_;
  int difop_fd_;
};
}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
