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
#define private public
#include "modules/localization/msf/common/util/timer.h"
#include <gtest/gtest.h>

namespace apollo {
namespace localization {
namespace msf {

class TimerTestSuite : public ::testing::Test {
 protected:
  TimerTestSuite() {}
  virtual ~TimerTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};
/**@brief Test. */
TEST_F(TimerTestSuite, test_timer) {
  Timer timer;
  timer.Start();
  boost::posix_time::ptime start_time = timer.start_time_;
  timer.End("end");
  boost::posix_time::ptime end_time = timer.end_time_;
  boost::posix_time::ptime start_time_new = timer.start_time_;

  ASSERT_GE(end_time, start_time);
  ASSERT_GE(start_time_new, start_time);
}
TEST_F(TimerTestSuite, test_time_accumulator) {
  TimeAccumulator timer_accumulator;
  timer_accumulator.Start();
  boost::posix_time::ptime start_time = timer_accumulator.start_time_;
  timer_accumulator.End();
  boost::posix_time::ptime start_time_new = timer_accumulator.start_time_;
  boost::posix_time::time_duration duration = timer_accumulator.duration_;
  timer_accumulator.GetDuration("duration");

  ASSERT_GE(duration, boost::posix_time::time_duration());
  ASSERT_GE(start_time_new, start_time);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
