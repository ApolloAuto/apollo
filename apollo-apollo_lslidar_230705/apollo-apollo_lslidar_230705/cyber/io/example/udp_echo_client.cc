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

using apollo::cyber::io::Session;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <server port>" << std::endl;
    return -1;
  }

  apollo::cyber::Init(argv[0]);

  int server_port = atoi(argv[1]);
  apollo::cyber::scheduler::Instance()->CreateTask(
      [&server_port]() {
        struct sockaddr_in server_addr;
        server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons((uint16_t)server_port);

        std::string user_input;
        std::vector<char> server_reply(2049);
        ssize_t nbytes = 0;
        uint32_t count = 0;

        Session session;
        session.Socket(AF_INET, SOCK_DGRAM, 0);
        if (session.Connect((struct sockaddr*)&server_addr,
                            sizeof(server_addr)) < 0) {
          std::cout << "connect to server failed, " << strerror(errno)
                    << std::endl;
          return;
        }

        while (true) {
          count = 0;
          std::cout << "please enter a message (enter Ctrl+C to exit):"
                    << std::endl;
          std::getline(std::cin, user_input);
          if (!apollo::cyber::OK()) {
            break;
          }
          if (user_input.empty()) {
            continue;
          }

          if (session.Send(user_input.c_str(), user_input.length(), 0) < 0) {
            std::cout << "send message failed." << std::endl;
            return;
          }

          while ((nbytes = session.Recv(server_reply.data(),
                                        server_reply.size(), 0)) > 0) {
            for (auto itr = server_reply.begin();
                 itr < server_reply.begin() + nbytes; ++itr) {
              std::cout << *itr;
            }
            count += (uint32_t)nbytes;
            if (count >= user_input.length()) {
              break;
            }
          }

          if (nbytes == 0) {
            std::cout << "server has been closed." << std::endl;
            session.Close();
            return;
          }

          if (nbytes < 0) {
            std::cout << "receive message from server failed." << std::endl;
            session.Close();
            return;
          }

          std::cout << std::endl;
        }
      },
      "echo_client");

  apollo::cyber::WaitForShutdown();
  return 0;
}
