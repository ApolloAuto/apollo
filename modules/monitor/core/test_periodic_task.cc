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

#include <thread>

#include "gtest/gtest.h"

#include "periodic_task.h"

namespace apollo {
namespace monitor {
namespace sysmon {


using std::chrono::system_clock;

// A function that runs n times before it stops with interval intv_ns
// between runs.
template<int n, int intv_ns>
void test_run(int *count, PeriodicTask *th)
{
  static system_clock::time_point t_last = system_clock::time_point::min();
  static std::chrono::nanoseconds t_diff_max =
      std::chrono::nanoseconds(intv_ns + 10000);
  static std::chrono::nanoseconds t_diff_min =
      std::chrono::nanoseconds(intv_ns - 10000);

  DBG_ONLY(
    ADEBUG << "run " << *count + 1 << "(until " << n << ")"
  );

  system_clock::time_point t_now = system_clock::now();
  if (t_last > system_clock::time_point::min()) {
    // Time difference between runs.
    std::chrono::nanoseconds t_diff = t_now - t_last;
    // It should be within (-10us, +10us) range.
    EXPECT_TRUE(t_diff < t_diff_max);
    EXPECT_TRUE(t_diff > t_diff_min);

    t_last = t_now;
  }
  ++(*count);
  if ((*count) >= n) {
    DBG_ONLY(
      ADEBUG << "stopping\n"
    );
    th->stop();
  }
}

TEST(PeriodicTaskTest, RunTest) {
  int count = 0;
  // Crete a task that will stop itself after 5 times,
  // runs with interval of 1 second.
  PeriodicRunner pt(1*NANOS_IN_SEC,
      std::bind(test_run<5, 1*NANOS_IN_SEC>, &count, &pt));
  std::thread tr(pt.get_runner());
  tr.join();

  // The task should have been executed 5 times.
  EXPECT_EQ(5, count);
}

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo
