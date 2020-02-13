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

#include "cyber/io/poller.h"

#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include "gtest/gtest.h"

#include "cyber/init.h"

namespace apollo {
namespace cyber {
namespace io {

TEST(PollerTest, operation) {
  auto poller = Poller::Instance();
  ASSERT_NE(poller, nullptr);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // invalid input
  PollRequest request;
  EXPECT_FALSE(poller->Register(request));

  int pipe_fd[2] = {-1, -1};
  ASSERT_EQ(pipe(pipe_fd), 0);
  ASSERT_EQ(fcntl(pipe_fd[0], F_SETFL, O_NONBLOCK), 0);
  ASSERT_EQ(fcntl(pipe_fd[1], F_SETFL, O_NONBLOCK), 0);

  // invalid input, callback is nullptr
  request.fd = pipe_fd[0];
  request.events = EPOLLIN | EPOLLET;
  request.timeout_ms = 0;
  EXPECT_FALSE(poller->Register(request));

  // timeout_ms is 0
  PollResponse response(123);
  request.callback = [&response](const PollResponse& rsp) { response = rsp; };
  EXPECT_TRUE(poller->Register(request));
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(response.events, 0);

  // timeout_ms is 50
  response.events = 123;
  request.timeout_ms = 50;
  EXPECT_TRUE(poller->Register(request));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(response.events, 123);
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  EXPECT_EQ(response.events, 0);

  // timeout_ms is 200
  response.events = 123;
  request.timeout_ms = 200;
  EXPECT_TRUE(poller->Register(request));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(response.events, 123);
  char msg = 'C';
  ssize_t res = 0;
  int try_num = 3;
  do {
    --try_num;
    res = write(pipe_fd[1], &msg, 1);
  } while (res <= 0 && try_num >= 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_NE(response.events & EPOLLIN, 0);

  EXPECT_TRUE(poller->Unregister(request));
  EXPECT_FALSE(poller->Unregister(request));
  request.callback = nullptr;
  EXPECT_FALSE(poller->Unregister(request));
  request.fd = -1;
  EXPECT_FALSE(poller->Unregister(request));

  poller->Shutdown();
  request.fd = pipe_fd[0];
  request.events = EPOLLIN | EPOLLET;
  request.timeout_ms = 0;
  request.callback = [](const PollResponse&) {};
  // poller has been shutdown
  EXPECT_FALSE(poller->Register(request));
  EXPECT_FALSE(poller->Unregister(request));

  close(pipe_fd[0]);
  close(pipe_fd[1]);
}

}  // namespace io
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  return RUN_ALL_TESTS();
}
