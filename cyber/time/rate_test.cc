/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include "cyber/time/rate.h"

#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <cstdint>

#include "gtest/gtest.h"

namespace {
// Keep simulated clock jumps local to the thread exercising Rate.
thread_local int64_t wall_offset_seconds = 0;
thread_local int realtime_reads = 0;
}  // namespace

// Interpose only this test executable's clock reads. The kernel clock and
// CLOCK_MONOTONIC remain unchanged, so no clock-setting privileges are needed.
extern "C" int clock_gettime(clockid_t clock_id,
                             struct timespec* value) noexcept {
  const int result =
      static_cast<int>(syscall(SYS_clock_gettime, clock_id, value));
  if (result == 0 && clock_id == CLOCK_REALTIME) {
    value->tv_sec += wall_offset_seconds;
    ++realtime_reads;
  }
  return result;
}

namespace apollo {
namespace cyber {

class RateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    wall_offset_seconds = 0;
    realtime_reads = 0;
  }

  void TearDown() override { wall_offset_seconds = 0; }
};

TEST_F(RateTest, constructors) {
  Rate frequency(50.0);
  Rate nanoseconds(uint64_t{20000000});
  Rate duration(Duration(0.02));
  // Snapshot the counter before assertions can invoke the test framework.
  const int reads = realtime_reads;
  EXPECT_EQ(0, reads);
  EXPECT_EQ(20000000, frequency.ExpectedCycleTime().ToNanosecond());
  EXPECT_EQ(frequency.ExpectedCycleTime(), nanoseconds.ExpectedCycleTime());
  EXPECT_EQ(frequency.ExpectedCycleTime(), duration.ExpectedCycleTime());
  EXPECT_EQ(0, duration.CycleTime().ToNanosecond());
}

TEST_F(RateTest, sleep) {
  const Time start = Time::MonoTime();
  Rate rate(Duration(0.02));
  rate.Sleep();
  EXPECT_GE(Time::MonoTime() - start, rate.ExpectedCycleTime());
  rate.Sleep();
  EXPECT_GE(Time::MonoTime() - start, rate.ExpectedCycleTime() * 2);
}

class RateClockJumpTest : public RateTest,
                          public ::testing::WithParamInterface<int64_t> {};

TEST_P(RateClockJumpTest, sleep_after_wall_clock_jump) {
  const Time wall_start = Time::Now();
  const Time start = Time::MonoTime();
  Rate rate(Duration(0.02));
  wall_offset_seconds = GetParam();
  // Verify that the test clock actually moved before exercising Rate.
  EXPECT_NEAR(GetParam(), (Time::Now() - wall_start).ToSecond(), 1.0);
  realtime_reads = 0;
  rate.Sleep();
  const int reads = realtime_reads;
  EXPECT_EQ(0, reads);
  EXPECT_GE(Time::MonoTime() - start, rate.ExpectedCycleTime());
  EXPECT_GE(rate.CycleTime().ToNanosecond(), 0);
  EXPECT_LT(rate.CycleTime().ToSecond(), 1.0);
}

TEST_P(RateClockJumpTest, reset_after_wall_clock_jump) {
  Rate rate(Duration(0.02));
  wall_offset_seconds = GetParam();
  realtime_reads = 0;
  const Time start = Time::MonoTime();
  rate.Reset();
  rate.Sleep();
  const int reads = realtime_reads;
  EXPECT_EQ(0, reads);
  EXPECT_GE(Time::MonoTime() - start, rate.ExpectedCycleTime());
}

INSTANTIATE_TEST_CASE_P(WallClock, RateClockJumpTest,
                        ::testing::Values(int64_t{-3600}, int64_t{3600}));

TEST_F(RateTest, cycle_overrun) {
  Rate rate(Duration(0.005));
  // Miss more than one cycle, then verify that Reset starts a fresh period.
  Duration(0.02).Sleep();
  rate.Sleep();
  EXPECT_GE(rate.CycleTime(), rate.ExpectedCycleTime() * 2);
  const Time start = Time::MonoTime();
  rate.Reset();
  rate.Sleep();
  EXPECT_GE(Time::MonoTime() - start, rate.ExpectedCycleTime());
}

}  // namespace cyber
}  // namespace apollo
