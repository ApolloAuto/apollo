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

#include <arpa/inet.h>
#include <fcntl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <linux/sockios.h>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>

#include "cyber/cyber.h"
#include "modules/drivers/lidar/hesai/parser/tcp_cmd_client.h"

namespace apollo {
namespace drivers {
namespace hesai {

int TcpCmdClient::BuildCmdHeader(const Command& cmd, unsigned char* buffer) {
  int index = 0;
  buffer[index++] = 0x47;
  buffer[index++] = 0x74;
  buffer[index++] = cmd.header.cmd;
  buffer[index++] = cmd.header.ret_code;  // color or mono
  buffer[index++] = (cmd.header.len >> 24) & 0xff;
  buffer[index++] = (cmd.header.len >> 16) & 0xff;
  buffer[index++] = (cmd.header.len >> 8) & 0xff;
  buffer[index++] = (cmd.header.len >> 0) & 0xff;
  return index;
}

bool TcpCmdClient::GetCalibration(std::string* content) {
  std::lock_guard<std::mutex> lck(mutex_);
  if (!Open()) {
    return false;
  }
  Command cmd;
  memset(&cmd, 0, sizeof(Command));
  cmd.header.cmd = PTC_COMMAND_GET_LIDAR_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = NULL;
  if (!WriteCmd(cmd)) {
    Close();
    return false;
  }

  Command feedback;
  if (!ReadCmd(&feedback)) {
    Close();
    return false;
  }

  *content = std::string(reinterpret_cast<char*>(feedback.data));

  Close();
  return true;
}

void TcpCmdClient::ParseHeader(const unsigned char* buffer, const int len,
                               CommandHeader* header) {
  int index = 0;
  header->cmd = buffer[index++];
  header->ret_code = buffer[index++];
  header->len =
      ((buffer[index] & 0xff) << 24) | ((buffer[index + 1] & 0xff) << 16) |
      ((buffer[index + 2] & 0xff) << 8) | ((buffer[index + 3] & 0xff) << 0);
}

bool TcpCmdClient::WriteCmd(const Command& cmd) {
  unsigned char buffer[128];
  int size = BuildCmdHeader(cmd, buffer);
  int ret = write(socket_fd_, buffer, size);
  if (ret != size) {
    AERROR << "write header error";
    return false;
  }
  if (cmd.header.len > 0 && cmd.data) {
    ret = write(socket_fd_, cmd.data, cmd.header.len);
    int len = static_cast<int>(cmd.header.len);
    if (ret != len) {
      AERROR << "write data error";
      return false;
    }
  }

  AINFO << "write cmd success";

  return true;
}

bool TcpCmdClient::ReadCmd(Command* feedback) {
  if (feedback == nullptr) {
    return false;
  }
  unsigned char buffer[1500];
  memset(buffer, 0, 10 * sizeof(char));
  int ret = Read(buffer, 2);
  if (ret <= 0 || buffer[0] != 0x47 || buffer[1] != 0x74) {
    AERROR << "server read header failed";
    return false;
  }

  ret = Read(buffer + 2, 6);
  if (ret != 6) {
    AERROR << "server read header failed";
    return false;
  }

  ParseHeader(buffer + 2, 6, &feedback->header);

  if (feedback->header.len == 0) {
    AINFO << "no data response, only header return";
    return false;
  }

  feedback->data = (unsigned char*)malloc(feedback->header.len + 1);
  if (!feedback->data) {
    AERROR << "malloc data error, oom";
    return false;
  }
  memset(feedback->data, 0, feedback->header.len + 1);

  ret = Read(feedback->data, feedback->header.len);
  if (ret != static_cast<int>(feedback->header.len)) {
    AERROR << "server read data failed";
    return false;
  }

  AINFO << "read data success, size:" << feedback->header.len;
  return true;
}

int TcpCmdClient::Read(unsigned char* buffer, int n) {
  int nleft = -1, nread = -1;
  unsigned char* ptr = buffer;
  nleft = n;
  while (nleft > 0) {
    if ((nread = read(socket_fd_, ptr, nleft)) < 0) {
      if (errno == EINTR) {
        nread = 0;
      } else {
        AERROR << "read() buffer error " << strerror(errno);
        return -1;
      }
    } else if (nread == 0) {
      break;
    }
    nleft -= nread;
    ptr += nread;
  }
  return n - nleft;
}

void TcpCmdClient::Close() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool TcpCmdClient::Open() {
  int sockfd = -1;
  struct sockaddr_in servaddr;
  bzero(&servaddr, sizeof(servaddr));
  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    AERROR << "socket() error " << strerror(errno);
    return false;
  }
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(port_);
  if (inet_pton(AF_INET, ip_.c_str(), &servaddr.sin_addr) <= 0) {
    AERROR << "inet_pton() error, ip:" << ip_;
    close(sockfd);
    return false;
  }
  if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
    close(sockfd);
    AERROR << "connect() error " << strerror(errno);
    return false;
  }
  socket_fd_ = sockfd;
  AINFO << "connect success";
  return true;
}

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
