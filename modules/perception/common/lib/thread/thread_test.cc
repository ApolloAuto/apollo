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

#include "modules/perception/common/lib/thread/thread.h"

#include <thread>
#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace lib {

class MyThread : public Thread {
 public:
  MyThread() : Thread(true), value_(0) {}
  virtual ~MyThread() {}

  int get_value() const { return value_; }

  MyThread(const MyThread &) = delete;
  MyThread &operator=(const MyThread &) = delete;

 protected:
  void Run() override {
    int count = 0;
    while (count < 10) {
      value_ += 10;
      ++count;
    }
  }

 private:
  int value_;
};

TEST(TestThread, Test) {
  MyThread my_thread;
  EXPECT_EQ(my_thread.get_value(), 0);
  my_thread.set_thread_name("my_thread");
  EXPECT_EQ(my_thread.get_thread_name(), "my_thread");

  my_thread.Start();
  my_thread.Join();
  EXPECT_EQ(my_thread.get_value(), 100);
  EXPECT_FALSE(my_thread.IsAlive());
  MyThread my_thread2;
  my_thread2.Start();
  EXPECT_TRUE(my_thread2.IsAlive());
  my_thread2.Join();
  EXPECT_EQ(my_thread2.tid(), 0);

  MyThread my_thread3;
  my_thread3.set_joinable(false);
  my_thread3.Start();
  my_thread3.set_joinable(false);
  EXPECT_TRUE(my_thread3.IsAlive());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(my_thread3.IsAlive());
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
