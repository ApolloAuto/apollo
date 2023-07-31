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

#include "modules/perception/common/lib/thread/concurrent_queue.h"

#include <thread>

#include "cyber/common/log.h"
#include "gtest/gtest.h"
#include "modules/perception/common/lib/thread/thread.h"

namespace apollo {
namespace perception {
namespace lib {

class PushThread : public Thread {
 public:
  explicit PushThread(ConcurrentQueue<int>* queue)
      : Thread(true), queue_(queue) {}
  virtual ~PushThread() {}

 protected:
  void Run() override {
    for (int idx = 1; idx < 10; ++idx) {
      queue_->Push(idx);
      AINFO << "PushThread push value: " << idx;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    queue_->Push(0);

    for (int idx = 10; idx < 15; ++idx) {
      queue_->Push(idx);
      AINFO << "PushThread push value: " << idx;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    AINFO << "PushThread finished.";
  }

 private:
  ConcurrentQueue<int>* queue_;
};

class PopThread : public Thread {
 public:
  explicit PopThread(ConcurrentQueue<int>* queue)
      : Thread(true), queue_(queue) {}
  virtual ~PopThread() {}

 protected:
  void Run() override {
    while (true) {
      int value = 0;
      queue_->Pop(&value);

      if (value == 0) {
        AINFO << "PopThread finished.";
        break;
      }

      AINFO << "PopThread pop value: " << value;
    }
  }

 private:
  ConcurrentQueue<int>* queue_;
};

class ConcurrentQueueTest : public testing::Test {
 protected:
  virtual void SetUp() {
    push_thread_ = new PushThread(&con_queue_);
    pop_thread_ = new PopThread(&con_queue_);
  }

  virtual void TearDown() {
    delete push_thread_;
    delete pop_thread_;
  }

 protected:
  ConcurrentQueue<int> con_queue_;
  PushThread* push_thread_;
  PopThread* pop_thread_;
};

TEST_F(ConcurrentQueueTest, TestAll) {
  push_thread_->Start();
  pop_thread_->Start();

  push_thread_->Join();
  pop_thread_->Join();
  EXPECT_EQ(con_queue_.Size(), 5);
  int data[] = {1, 2, 3};
  EXPECT_TRUE(con_queue_.TryPop(data));
  con_queue_.Clear();
  EXPECT_FALSE(con_queue_.TryPop(data));
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
