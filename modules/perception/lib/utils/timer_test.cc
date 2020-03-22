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

#include "modules/perception/lib/utils/timer.h"

#include <thread>
#include "gtest/gtest.h"

#include "modules/perception/lib/utils/perf.h"

namespace apollo {
namespace perception {
namespace lib {

TEST(TimerTest, Test) {
  Timer timer;
  timer.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const uint64_t elapsed_time = timer.End("TimerTest");
  EXPECT_GE(elapsed_time, 99);
  EXPECT_LE(elapsed_time, 101);
}

TEST(TimerWrapperTest, Test) {
  TimerWrapper wrapper("TimerWrapperTest");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

TEST(PerfFunctionTest, Test) {
  PERCEPTION_PERF_FUNCTION();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST(PerfBlockTest, Test) {
  PERCEPTION_PERF_BLOCK_START();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  PERCEPTION_PERF_BLOCK_END("BLOCK1");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  PERCEPTION_PERF_BLOCK_END("BLOCK2");
}

TEST(GetFunctionNameTest, Test) {
  std::string full_name =
      "void apollo::perception::"
      "lib::Thread::set_joinable(bool)";
  EXPECT_EQ(get_full_name(full_name),
            "apollo::perception::lib::Thread::set_joinable");
  full_name =
      "void apollo::perception::lib::Thread::"
      "set_joinable";
  EXPECT_EQ(get_full_name(full_name),
            "void apollo::perception::lib::Thread::"
            "set_joinable");
  full_name =
      "apollo::perception::lib::Thread::"
      "set_joinable()";
  EXPECT_EQ(get_full_name(full_name),
            "apollo::perception::lib::Thread::"
            "set_joinable()");
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
