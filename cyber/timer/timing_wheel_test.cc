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

#include "cyber/timer/timing_wheel.h"

#include <gtest/gtest.h>
#include <unistd.h>
#include <atomic>
#include <memory>

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/init.h"

namespace apollo {
namespace cyber {

class TestHandler {
 public:
  TestHandler() = default;

  void increment() { count_++; }

  uint64_t count() { return count_; }

 private:
  std::atomic<uint64_t> count_ = {0};
};

TEST(TimingWheelTest, Oneshot) {
  TimingWheel tw;
  std::shared_ptr<TestHandler> th(new TestHandler());
  ASSERT_EQ(0, th->count());
  std::function<void(void)> f = std::bind(&TestHandler::increment, th.get());
  tw.Step();
  tw.StartTimer(10, f, true);
  for (int i = 0; i < 10; i++) {
    tw.Step();
    usleep(1000);
  }
  ASSERT_EQ(1, th->count());
}

TEST(TimingWheelTest, Period) {
  TimingWheel tw;
  std::shared_ptr<TestHandler> th(new TestHandler());
  ASSERT_EQ(0, th->count());
  std::function<void(void)> f = std::bind(&TestHandler::increment, th.get());
  tw.Step();
  tw.StartTimer(10, f, false);

  for (uint64_t i = 0; i < 100; i++) {
    tw.Step();
    if ((i + 1) % 10 == 0) {
      usleep(10 * 1000);
      ASSERT_TRUE(i <= th->count() && i + 1 >= th->count());
    }
  }
}

}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  auto res = RUN_ALL_TESTS();
  return res;
}
