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

#include "cyber/time/time.h"

#include <iostream>

#include "gtest/gtest.h"

#include "cyber/time/duration.h"

namespace apollo {
namespace cyber {

TEST(TimeTest, constructor) {
  Time time(100UL);
  EXPECT_EQ(100UL, time.ToNanosecond());

  time = Time(1.1);
  EXPECT_EQ(1100000000UL, time.ToNanosecond());
  EXPECT_DOUBLE_EQ(1.1, time.ToSecond());

  time = Time(1, 1);
  EXPECT_EQ(1000000001UL, time.ToNanosecond());
  EXPECT_DOUBLE_EQ(1.000000001, time.ToSecond());

  Time time2(time);
  EXPECT_EQ(time, time2);
}

TEST(TimeTest, operators) {
  Time t1(100);
  Duration d(200);
  Time t2(300);
  EXPECT_NE(t1, t2);
  EXPECT_LT(t1, t2);
  EXPECT_LE(t1, t2);
  EXPECT_GT(t2, t1);
  EXPECT_GE(t2, t1);
  EXPECT_EQ(t1 + d, t2);
  EXPECT_EQ(t2 - d, t1);
  EXPECT_EQ(t1 += d, t2);
  EXPECT_GE(t1, t2);
  EXPECT_LE(t1, t2);
  EXPECT_EQ(Time(100), t1 -= d);
}

TEST(TimeTest, to_string) {
  Time t1(1531225311123456789UL);
  std::cout << t1.ToString().c_str() << std::endl;
  Time t2(1531225311000006789UL);
  std::cout << t2.ToString().c_str() << std::endl;
}

TEST(TimeTest, now) { std::cout << "Time Now: " << Time::Now() << std::endl; }

TEST(TimeTest, is_zero) {
  Time time;
  EXPECT_TRUE(time.IsZero());
  EXPECT_FALSE(Time::MAX.IsZero());
  EXPECT_TRUE(Time::MIN.IsZero());
}

}  // namespace cyber
}  // namespace apollo
