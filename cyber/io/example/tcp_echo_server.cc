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
  std::vector<char> recv_buffer(2049);
  int nbytes = 0;
  while ((nbytes = static_cast<int>(
              session->Recv(recv_buffer.data(), recv_buffer.size(), 0))) > 0) {
    session->Write(recv_buffer.data(), nbytes);
  }

  if (nbytes == 0) {
    std::cout << "client has been closed." << std::endl;
    session->Close();
  }

  if (nbytes < 0) {
    std::cout << "receive from client failed." << std::endl;
    session->Close();
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

        Session session;
        session.Socket(AF_INET, SOCK_STREAM, 0);
        if (session.Bind((struct sockaddr*)&server_addr, sizeof(server_addr)) <
            0) {
          std::cout << "bind to port[" << server_port << "] failed."
                    << std::endl;
          return;
        }
        session.Listen(10);
        auto conn_session = session.Accept((struct sockaddr*)nullptr, nullptr);
        std::cout << "accepted" << std::endl;
        auto routine_name =
            "connected session" + std::to_string(Time::Now().ToNanosecond());
        apollo::cyber::scheduler::Instance()->CreateTask(
            std::bind(Echo, conn_session), routine_name);
      },
      "echo_server");

  apollo::cyber::WaitForShutdown();
  return 0;
}
