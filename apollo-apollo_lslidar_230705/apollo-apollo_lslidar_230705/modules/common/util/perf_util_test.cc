/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/util/perf_util.h"

#include <thread>

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace util {

TEST(TimeTest, test_timer) {
  Timer timer;
  timer.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const uint64_t elapsed_time = timer.End("TimerTest");
  EXPECT_GE(elapsed_time, 90);
  EXPECT_LE(elapsed_time, 110);
}

TEST(TimerWrapperTest, test) {
  TimerWrapper wrapper("TimerWrapperTest");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

TEST(PerfFunctionTest, test) {
  PERF_FUNCTION_WITH_NAME("FunctionTest");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST(PerfBlockTest, test) {
  PERF_BLOCK_START();
  // do somethings.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  PERF_BLOCK_END("BLOCK1");

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  PERF_BLOCK_END("BLOCK2");
}

}  // namespace util
}  // namespace common
}  // namespace apollo
