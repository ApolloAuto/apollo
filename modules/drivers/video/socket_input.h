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

#include <unistd.h>
#include <stdio.h>
//#include <pcap.h>

#include "cyber/cyber.h"
#include "modules/drivers/proto/sensor_image.pb.h"

#include "modules/drivers/video/input.h"

namespace apollo {
namespace drivers {
namespace video {

static const int SOCKET_TIMEOUT = -2;
static const int RECIEVE_FAIL = -3;
static const int POLL_TIMEOUT = 1000;  // one second (in msec)
static const size_t H265_FRAME_PACKAGE_SIZE = 4*1024*1024;
static const size_t H265_PDU_SIZE = 1500;


using apollo::drivers::CompressedImage;

/** @brief Live Velodyne input from socket. */
class SocketInput{
public:
  SocketInput();
  virtual ~SocketInput();
  void Init(uint32_t port);
  int get_frame_packet(std::shared_ptr<CompressedImage>& h265);

private:
  int _sockfd;
  int _port;
  uint8_t *_buf = nullptr;
  uint8_t *_pdu = nullptr;
  int _pkgNum;
  int _bytesNum;
  uint32_t _frame_id;
  bool input_available(int timeout);
  std::map<int, std::string> _fpd_frameid_map;
};

} //video
} //drivers
} //apollo


