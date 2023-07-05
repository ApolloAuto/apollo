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

#ifndef LIDAR_HESAI_SRC_TCP_CMD_CLIENT_H_
#define LIDAR_HESAI_SRC_TCP_CMD_CLIENT_H_

#include <mutex>
#include <string>

namespace apollo {
namespace drivers {
namespace hesai {

typedef enum {
  PTC_COMMAND_GET_CALIBRATION = 0,
  PTC_COMMAND_SET_CALIBRATION,
  PTC_COMMAND_HEARTBEAT,
  PTC_COMMAND_RESET_CALIBRATION,
  PTC_COMMAND_TEST,
  PTC_COMMAND_GET_LIDAR_CALIBRATION,
} PTC_COMMAND;

typedef struct CommandHeader {
  unsigned char cmd = '\0';
  unsigned char ret_code = '\0';
  uint32_t len = 0;
} CommandHeader;

typedef struct Command {
  CommandHeader header;
  unsigned char* data = nullptr;
  ~Command() {
    if (data != nullptr) {
      free(data);
    }
  }
} Command;

class TcpCmdClient {
 public:
  TcpCmdClient(const std::string& ip, uint32_t port) : ip_(ip), port_(port) {}
  ~TcpCmdClient() { Close(); }
  bool GetCalibration(std::string* content);

 private:
  bool Open();
  void Close();
  bool ReadCmd(Command* feedBack);
  int Read(unsigned char* buffer, int n);
  bool WriteCmd(const Command& cmd);
  int BuildCmdHeader(const Command& cmd, unsigned char* buffer);
  void ParseHeader(const unsigned char* buffer, const int len,
                   CommandHeader* header);
  std::string ip_;
  uint32_t port_ = 0;
  int socket_fd_ = -1;
  std::mutex mutex_;
};

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

#endif
