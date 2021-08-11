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
#include "modules/drivers/lidar/robosense/input/input.h"
namespace apollo {
namespace drivers {

namespace robosense {
Input::Input(const uint16_t &msop_port, const uint16_t &difop_port) {
  this->msop_fd_ = setUpSocket(msop_port);
  this->difop_fd_ = setUpSocket(difop_port);
}

Input::~Input() {
  close(this->msop_fd_);
  close(this->difop_fd_);
}

int Input::setUpSocket(uint16_t port) {
  int sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) {
    std::cerr << "socket: " << std::strerror(errno) << std::endl;
    return -1;
  }
  struct sockaddr_in my_addr;
  memset(reinterpret_cast<char *>(&my_addr), 0, sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(port);
  my_addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock_fd, (struct sockaddr *)&my_addr, sizeof(my_addr)) < 0) {
    std::cerr << "bind: " << std::strerror(errno) << std::endl;
    return -1;
  }
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) <
      0) {
    std::cerr << "setsockopt: " << std::strerror(errno) << std::endl;
    return -1;
  }
  return sock_fd;
}

InputState Input::getPacket(uint8_t *pkt, uint32_t timeout) {
  InputState res = InputState(0);
  if (pkt == NULL) {
    return INPUT_ERROR;
  }
  fd_set rfds;
  struct timeval tmout;
  tmout.tv_sec = timeout / 1000;
  tmout.tv_usec = (timeout % 1000) * 1000;
  FD_ZERO(&rfds);
  FD_SET(this->msop_fd_, &rfds);
  FD_SET(this->difop_fd_, &rfds);
  int max_fd = std::max(this->msop_fd_, this->difop_fd_);
  int retval = select(max_fd + 1, &rfds, NULL, NULL, &tmout);
  if (retval == -1 && errno == EINTR) {
    res = INPUT_EXIT;
  } else if (retval == -1) {
    std::cerr << "select: " << std::strerror(errno) << std::endl;
    res = InputState(res | INPUT_ERROR);
  } else if (retval) {
    ssize_t n;
    if (FD_ISSET(this->msop_fd_, &rfds)) {
      res = InputState(res | INPUT_MSOP);
      n = recvfrom(this->msop_fd_, pkt, RSLIDAR_PKT_LEN, 0, NULL, NULL);
    } else if (FD_ISSET(this->difop_fd_, &rfds)) {
      res = InputState(res | INPUT_DIFOP);
      n = recvfrom(this->difop_fd_, pkt, RSLIDAR_PKT_LEN, 0, NULL, NULL);
    } else {
      return INPUT_ERROR;
    }
    if (n != RSLIDAR_PKT_LEN) {
      res = InputState(res | INPUT_ERROR);
    }
  }
  return res;
}

}  // namespace robosense
}  // namespace drivers
}  // namespace apollo
