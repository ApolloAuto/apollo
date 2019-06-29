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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <sys/socket.h>

#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "modules/bridge/common/bridge_proto_serialized_buf.h"
#include "modules/canbus/proto/chassis.pb.h"

bool send(const std::string &remote_ip, uint16_t remote_port) {
  for (int i = 0; i < 100; i++) {
    auto pb_msg = std::make_shared<apollo::canbus::Chassis>();
    pb_msg->set_engine_started(true);
    pb_msg->set_engine_rpm(static_cast<float>(i * 2.0));
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(remote_port);

    std::cout << "connecting to server... " << std::endl;

    int sock_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

    int res =
        connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (res < 0) {
      std::cout << "connected server failed " << std::endl;
      continue;
    }

    std::cout << "connected to server success. port [" << remote_port << "]"
              << std::endl;

    apollo::bridge::BridgeProtoSerializedBuf<apollo::canbus::Chassis> proto_buf;
    proto_buf.Serialize(pb_msg, "Chassis");
    for (size_t j = 0; j < proto_buf.GetSerializedBufCount(); j++) {
      ssize_t nbytes = send(sock_fd, proto_buf.GetSerializedBuf(j),
                            proto_buf.GetSerializedBufSize(j), 0);
      if (nbytes != static_cast<ssize_t>(proto_buf.GetSerializedBufSize(j))) {
        std::cout << "sent msg failed " << std::endl;
        break;
      }
      std::cout << "sent " << nbytes << "bytes to server" << std::endl;
    }
    close(sock_fd);
  }
  return true;
}

int main(int argc, char *argv[]) {
  send("127.0.0.1", 8901);
  return 0;
}
