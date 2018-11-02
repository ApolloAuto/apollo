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

#include <gtest/gtest.h>
#include <string>

#include "cyber/cyber.h"
#include "cyber/io/poll_data.h"
#include "cyber/io/poll_handler.h"
#include "cyber/io/poller.h"

namespace apollo {
namespace cyber {
namespace io {

TEST(PollTest, constructor) {
  Poller::Instance();
  int pipe_fd[2] = {-1, -1};
  pipe(pipe_fd);
  fcntl(pipe_fd[0], F_SETFL, O_NONBLOCK);
  fcntl(pipe_fd[1], F_SETFL, O_NONBLOCK);

  std::string name = "pipe_recv";
  cyber::scheduler::Scheduler::Instance()->CreateTask(
      [&]() {
        PollHandler handler(pipe_fd[0]);

        if (handler.Block(2000, true)) {
          std::cout << "block succ" << std::endl;
          char c = '0';
          while (read(pipe_fd[0], &c, 1) > 0) {
          }
          std::cout << "recv char: " << c << std::endl;
        } else {
          std::cout << "block failed" << std::endl;
        }
      },
      name);
  usleep(300000);

  char msg = 'C';
  write(pipe_fd[1], &msg, 1);

  sleep(5);
  Poller::Instance()->Shutdown();
}

}  // namespace io
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  apollo::cyber::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  auto res = RUN_ALL_TESTS();
  apollo::cyber::Shutdown();
  return res;
}
