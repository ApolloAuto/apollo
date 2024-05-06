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

#include "modules/perception/common/lib/thread/thread_worker.h"

#include <thread>
#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace lib {

TEST(ThreadWorkerTest, ThreadWorkerTest1) {
  int count = 0;
  ThreadWorker worker;
  worker.Bind([&]() {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return false;
  });
  worker.Start();
  for (int i = 0; i < 5; ++i) {
    worker.WakeUp();
    ++count;
    worker.Join();
  }
  worker.Release();
  EXPECT_EQ(count, 5);
}

TEST(ThreadWorkerTest, ThreadWorkerTest2) {
  int count = 0;
  ThreadWorker worker;
  worker.Bind([&]() {
    ++count;
    return true;
  });
  worker.Start();
  for (int i = 0; i < 5; ++i) {
    worker.WakeUp();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    worker.Join();
  }
  worker.Release();
  EXPECT_EQ(count, 5);
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
