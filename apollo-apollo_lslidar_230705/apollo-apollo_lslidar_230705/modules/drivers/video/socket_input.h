/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <unistd.h>

#include <cstdio>
#include <map>
#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"

namespace apollo {
namespace drivers {
namespace video {

static const int SOCKET_TIMEOUT = -2;
static const int RECEIVE_FAIL = -3;
static const int POLL_TIMEOUT = 1000;  // one second (in msec)
static const size_t H265_FRAME_PACKAGE_SIZE = 4 * 1024 * 1024;
static const size_t H265_PDU_SIZE = 1500;

/** @brief Live Velodyne input from socket. */
class SocketInput {
 public:
  SocketInput();
  virtual ~SocketInput();
  void Init(uint32_t port);
  int GetFramePacket(std::shared_ptr<CompressedImage> h265);

 private:
  int sockfd_;
  int port_;
  uint8_t *buf_ = nullptr;
  uint8_t *pdu_ = nullptr;
  int pkg_num_;
  int bytes_num_;
  uint32_t frame_id_;
  bool InputAvailable(int timeout);
};

}  // namespace video
}  // namespace drivers
}  // namespace apollo
