/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include <sys/socket.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"
#include "cyber/task/task.h"
#include "cyber/time/time.h"

using apollo::cyber::Time;
using apollo::cyber::io::Session;

void Echo(const std::shared_ptr<Session>& session) {
  struct sockaddr_in client_addr;
  std::vector<char> recv_buffer(2049);
  int nbytes = 0;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(client_addr));

  while (true) {
    nbytes = static_cast<int>(
        session->RecvFrom(recv_buffer.data(), recv_buffer.size(), 0,
                          (struct sockaddr*)&client_addr, &sock_len));
    if (nbytes < 0) {
      std::cout << "recv from client failed." << std::endl;
      continue;
    }
    session->SendTo(recv_buffer.data(), nbytes, 0,
                    (const struct sockaddr*)&client_addr, sock_len);
  }
}

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <server port>" << std::endl;
    return -1;
  }

  apollo::cyber::Init(argv[0]);

  uint16_t server_port = static_cast<uint16_t>(atoi(argv[1]));
  apollo::cyber::scheduler::Instance()->CreateTask(
      [&server_port]() {
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = htons(INADDR_ANY);
        server_addr.sin_port = htons(server_port);

        auto session = std::make_shared<Session>();
        session->Socket(AF_INET, SOCK_DGRAM, 0);
        if (session->Bind((struct sockaddr*)&server_addr, sizeof(server_addr)) <
            0) {
          std::cout << "bind to port[" << server_port << "] failed."
                    << std::endl;
          return;
        }
        Echo(session);
        session->Close();
      },
      "echo_server");

  apollo::cyber::WaitForShutdown();
  return 0;
}
