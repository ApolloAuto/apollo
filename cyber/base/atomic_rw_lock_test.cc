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

#include "cyber/base/atomic_rw_lock.h"

#include <thread>

#include "gtest/gtest.h"

#include "cyber/base/reentrant_rw_lock.h"

namespace apollo {
namespace cyber {
namespace base {

TEST(ReentrantRWLockTest, read_lock) {
  int count = 0;
  int thread_init = 0;
  bool flag = true;
  ReentrantRWLock lock;
  auto f = [&]() {
    ReadLockGuard<ReentrantRWLock> lg(lock);
    count++;
    thread_init++;
    while (flag) {
      std::this_thread::yield();
    }
  };
  std::thread t1(f);
  std::thread t2(f);
  while (thread_init != 2) {
    std::this_thread::yield();
  }
  EXPECT_EQ(2, count);
  flag = false;
  t1.join();
  t2.join();
  {
    ReadLockGuard<ReentrantRWLock> lg1(lock);
    {
      ReadLockGuard<ReentrantRWLock> lg2(lock);
      {
        ReadLockGuard<ReentrantRWLock> lg3(lock);
        { ReadLockGuard<ReentrantRWLock> lg4(lock); }
      }
    }
  }
}

TEST(ReentrantRWLockTest, write_lock) {
  int count = 0;
  int thread_run = 0;
  bool flag = true;
  ReentrantRWLock lock(false);
  auto f = [&]() {
    thread_run++;
    WriteLockGuard<ReentrantRWLock> lg(lock);
    count++;
    while (flag) {
      std::this_thread::yield();
    }
  };
  std::thread t1(f);
  std::thread t2(f);
  while (thread_run != 2) {
    std::this_thread::yield();
  }
  EXPECT_EQ(1, count);
  flag = false;
  t1.join();
  t2.join();

  {
    WriteLockGuard<ReentrantRWLock> lg1(lock);
    {
      WriteLockGuard<ReentrantRWLock> lg2(lock);
      { ReadLockGuard<ReentrantRWLock> lg3(lock); }
    }
  }
}

}  // namespace base
}  // namespace cyber
}  // namespace apollo
