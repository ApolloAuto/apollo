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

#include "cyber/time/duration.h"

#include "gtest/gtest.h"

#include "cyber/time/time.h"

namespace apollo {
namespace cyber {

TEST(DurationTest, constructor) {
  Duration duration(100);
  EXPECT_EQ(100, duration.ToNanosecond());

  duration = Duration(1.1);
  EXPECT_EQ(1100000000UL, duration.ToNanosecond());
  EXPECT_DOUBLE_EQ(1.1, duration.ToSecond());

  duration = Duration(1, 1);
  EXPECT_EQ(1000000001UL, duration.ToNanosecond());
  EXPECT_DOUBLE_EQ(1.000000001, duration.ToSecond());

  Duration d2(duration);
  EXPECT_EQ(duration, d2);
}

TEST(DurationTest, operators) {
  Duration d1(100);
  Duration d2(200);
  Duration d3(300);
  EXPECT_NE(d1, d3);
  EXPECT_LT(d1, d3);
  EXPECT_LE(d1, d3);
  EXPECT_GT(d3, d1);
  EXPECT_GE(d3, d1);
  EXPECT_EQ(d1 + d2, d3);
  EXPECT_EQ(d2, d1 * 2);
  EXPECT_EQ(d3 - d2, d1);
  EXPECT_EQ(d1 += d2, d3);
  EXPECT_GE(d1, d3);
  EXPECT_LE(d1, d3);
  EXPECT_EQ(Duration(100), d1 -= d2);
  EXPECT_EQ(d2, d1 *= 2);
}

TEST(DurationTest, is_zero) {
  Duration duration;
  EXPECT_TRUE(duration.IsZero());
}

}  // namespace cyber
}  // namespace apollo
