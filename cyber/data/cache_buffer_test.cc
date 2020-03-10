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

#include "cyber/data/cache_buffer.h"

#include <mutex>
#include <thread>
#include <utility>
#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace data {

TEST(CacheBufferTest, cache_buffer_test) {
  CacheBuffer<int> buffer(32);
  EXPECT_TRUE(buffer.Empty());
  for (int i = 0; i < 32 - 1; i++) {
    buffer.Fill(std::move(i));
    EXPECT_FALSE(buffer.Full());
    EXPECT_EQ(i, buffer[i + 1]);
    EXPECT_EQ(i, buffer.at(i + 1));
  }
  EXPECT_EQ(31, buffer.Size());
  EXPECT_EQ(1, buffer.Head());
  EXPECT_EQ(31, buffer.Tail());
  EXPECT_EQ(0, buffer.Front());
  EXPECT_EQ(30, buffer.Back());
  buffer.Fill(31);
  EXPECT_TRUE(buffer.Full());
  EXPECT_EQ(32, buffer.Size());

  CacheBuffer<int> buffer1(std::move(buffer));
  EXPECT_EQ(buffer.Size(), buffer1.Size());
  EXPECT_EQ(buffer.Head(), buffer1.Head());
  EXPECT_EQ(buffer.Tail(), buffer1.Tail());
  EXPECT_EQ(buffer.Front(), buffer1.Front());
  EXPECT_EQ(buffer.Back(), buffer1.Back());
  EXPECT_TRUE(buffer1.Full());
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo
