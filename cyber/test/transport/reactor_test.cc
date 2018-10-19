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

#include "gtest/gtest.h"

#include <sys/epoll.h>
#include <unistd.h>
#include <thread>

#include "cyber/common/log.h"
#include "cyber/transport/common/syscall_wrapper.h"
#include "cyber/transport/reactor/reactor.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(ReactorTest, constructor) {
  auto reactor_a = Reactor::Instance();
  auto reactor_b = Reactor::Instance();
  EXPECT_EQ(reactor_a, reactor_b);
}

TEST(ReactorTest, add_modify_delete) {
  auto reactor = Reactor::Instance();
  reactor->Start();

  int local_pipe[2] = {-1, -1};
  ASSERT_TRUE(pipe(local_pipe) == 0);
  ASSERT_TRUE(SetNonBlocking(local_pipe[0]));

  EXPECT_FALSE(reactor->Modify(local_pipe[0], EPOLLIN));
  EXPECT_FALSE(reactor->Delete(local_pipe[0]));

  char recv = '\0';

  EXPECT_TRUE(reactor->Add(local_pipe[0], EPOLLIN, nullptr));

  const char send = 'B';
  EXPECT_TRUE(write(local_pipe[1], &send, 1) > 0);
  std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(200));
  EXPECT_NE(recv, send);

  EXPECT_TRUE(reactor->Delete(local_pipe[0]));

  // clear pipe
  while (read(local_pipe[0], &recv, 1) > 0) {
  }

  auto handler = [&](const EventsType& events) {
    if (events & EPOLLIN) {
      if (read(local_pipe[0], &recv, 1) < 0) {
        AERROR << "read failed.";
      }
    }
  };

  EXPECT_TRUE(reactor->Add(local_pipe[0], 0, handler));

  recv = '\0';
  EXPECT_TRUE(write(local_pipe[1], &send, 1) > 0);
  std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(200));
  EXPECT_NE(recv, send);

  // clear pipe
  while (read(local_pipe[0], &recv, 1) > 0) {
  }

  EXPECT_TRUE(reactor->Modify(local_pipe[0], EPOLLIN));

  recv = '\0';
  EXPECT_TRUE(write(local_pipe[1], &send, 1) > 0);
  std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(200));
  EXPECT_EQ(recv, send);

  EXPECT_TRUE(reactor->Delete(local_pipe[0]));
  recv = '\0';
  EXPECT_TRUE(write(local_pipe[1], &send, 1) > 0);
  std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(200));
  EXPECT_NE(recv, send);

  // bad fd
  reactor->Add(-1, EPOLLIN, nullptr);
  reactor->Modify(-1, EPOLLOUT);
  reactor->Delete(-1);
}

TEST(ReactorTest, start) {
  auto reactor = Reactor::Instance();
  reactor->Start();
  // repeated call
  reactor->Start();
}

TEST(ReactorTest, shutdown) {
  auto reactor = Reactor::Instance();
  reactor->Shutdown();
  // repeated call
  reactor->Shutdown();
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
