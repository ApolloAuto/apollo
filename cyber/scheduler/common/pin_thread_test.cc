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

#include "cyber/scheduler/common/pin_thread.h"

#include <sys/syscall.h>
#include <string>
#include <vector>
#include "gtest/gtest.h"

#include "cyber/cyber.h"

namespace apollo {
namespace cyber {
namespace scheduler {

TEST(PinThreadTest, parse_cpuset) {
  std::string cpuset = "0-1,2-3";
  std::vector<int> cpus;
  ParseCpuset(cpuset, &cpus);
  ASSERT_EQ(cpus.size(), 4);
  cpuset = "0-1";
  cpus.clear();
  ParseCpuset(cpuset, &cpus);
  ASSERT_EQ(cpus.size(), 2);
  cpuset = "0";
  cpus.clear();
  ParseCpuset(cpuset, &cpus);
  ASSERT_EQ(cpus.size(), 1);
}

TEST(PinThreadTest, affinity) {
  std::string affinity = "range";
  std::vector<int> cpuset;
  cpuset.emplace_back(0);
  std::thread t = std::thread([]() {});
  SetSchedAffinity(&t, cpuset, affinity, 0);
  if (t.joinable()) {
    t.join();
  }
  affinity = "1to1";
  std::thread t1 = std::thread([]() {});
  SetSchedAffinity(&t1, cpuset, affinity, 0);
  if (t1.joinable()) {
    t1.join();
  }
}

TEST(pin_thread_test, sched_policy) {
  std::string policy = "SCHED_FIFO";
  int priority = 0;
  std::thread t = std::thread([]() {});
  SetSchedPolicy(&t, policy, priority);
  if (t.joinable()) {
    t.join();
  }

  policy = "SCHED_RR";
  std::thread t1 = std::thread([]() {});
  SetSchedPolicy(&t1, policy, priority);
  if (t1.joinable()) {
    t1.join();
  }

  policy = "SCHED_OTHER";
  std::atomic<pid_t> tid{-1};
  std::thread t2 =
      std::thread([&]() { tid = static_cast<int>(syscall(SYS_gettid)); });
  while (tid.load() == -1) {
    cpu_relax();
  }
  SetSchedPolicy(&t2, policy, priority, tid.load());
  if (t2.joinable()) {
    t2.join();
  }
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
