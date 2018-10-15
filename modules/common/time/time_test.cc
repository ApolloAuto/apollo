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

#include "modules/common/time/time.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace time {

TEST(TimeTest, DurationToMicros) {
  Duration duration = std::chrono::milliseconds(12);
  EXPECT_EQ(12000, AsInt64<micros>(duration));
}

TEST(TimeTest, DurationToMillis) {
  Duration duration = std::chrono::microseconds(1234567);
  EXPECT_EQ(1234, AsInt64<millis>(duration));
}

TEST(TimeTest, AsDouble) {
  Duration duration = std::chrono::microseconds(123456789012);
  EXPECT_DOUBLE_EQ(123456.789012, ToSecond(duration));
}

TEST(TimeTest, TimestampAsDouble) {
  Timestamp timestamp = FromInt64<nanos>(123456789012345);
  EXPECT_DOUBLE_EQ(123456.789012345, ToSecond(timestamp));
}

TEST(TimeTest, TimestampFromAndTo) {
  Timestamp timestamp = FromInt64<micros>(1234567);
  EXPECT_EQ(1234, AsInt64<millis>(timestamp));
}

TEST(TimeTest, TimestampFromAndToDouble) {
  Timestamp timestamp = From(1234567.889923456);
  EXPECT_DOUBLE_EQ(1234567.889923456, ToSecond(timestamp));
}

TEST(TimeTest, MockTime) {
  EXPECT_EQ(Clock::SYSTEM, Clock::mode());
  Clock::SetMode(Clock::MOCK);
  EXPECT_EQ(Clock::MOCK, Clock::mode());

  EXPECT_EQ(0, AsInt64<micros>(Clock::Now()));
  Clock::SetNow(micros(123));

  EXPECT_EQ(123, AsInt64<micros>(Clock::Now()));

  Clock::SetNowInSeconds(123.456);
  EXPECT_EQ(123.456, Clock::NowInSeconds());
}

}  // namespace time
}  // namespace common
}  // namespace apollo
