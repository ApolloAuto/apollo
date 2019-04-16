/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/time/timer.h"

#include <unistd.h>

#include "cyber/common/log.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace time {

TEST(TimeTest, test_timer) {
  Timer timer;
  timer.Start();
  usleep(100000);
  const uint64_t elapsed_time = timer.End("TimerTest");
  EXPECT_GE(elapsed_time, 90);
  EXPECT_LE(elapsed_time, 110);
}

TEST(TimerWrapperTest, test) {
  TimerWrapper wrapper("TimerWrapperTest");
  usleep(200000);
}

TEST(PerfFunctionTest, test) {
  PERF_FUNCTION("FunctionTest");
  usleep(100000);
}

TEST(PerfBlockTest, test) {
  PERF_BLOCK_START();
  // do somethings.
  usleep(100000);
  PERF_BLOCK_END("BLOCK1");

  usleep(200000);
  PERF_BLOCK_END("BLOCK2");
}

}  // namespace time
}  // namespace common
}  // namespace apollo
