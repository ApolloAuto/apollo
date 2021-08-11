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

#include "cyber/data/channel_buffer.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/common/util.h"

namespace apollo {
namespace cyber {
namespace data {

auto channel0 = common::Hash("/channel0");

TEST(ChannelBufferTest, Fetch) {
  auto cache_buffer = new CacheBuffer<std::shared_ptr<int>>(2);
  auto buffer = std::make_shared<ChannelBuffer<int>>(channel0, cache_buffer);
  std::shared_ptr<int> msg;
  uint64_t index = 0;
  EXPECT_FALSE(buffer->Fetch(&index, msg));
  buffer->Buffer()->Fill(std::make_shared<int>(1));
  EXPECT_TRUE(buffer->Fetch(&index, msg));
  EXPECT_EQ(1, *msg);
  EXPECT_EQ(1, index);
  index++;
  EXPECT_FALSE(buffer->Fetch(&index, msg));
  buffer->Buffer()->Fill(std::make_shared<int>(2));
  buffer->Buffer()->Fill(std::make_shared<int>(3));
  buffer->Buffer()->Fill(std::make_shared<int>(4));
  EXPECT_TRUE(buffer->Fetch(&index, msg));
  EXPECT_EQ(4, *msg);
  EXPECT_EQ(4, index);
  index++;
  EXPECT_FALSE(buffer->Fetch(&index, msg));
  EXPECT_EQ(4, *msg);
}

TEST(ChannelBufferTest, Latest) {
  auto cache_buffer = new CacheBuffer<std::shared_ptr<int>>(10);
  auto buffer = std::make_shared<ChannelBuffer<int>>(channel0, cache_buffer);
  std::shared_ptr<int> msg;
  EXPECT_FALSE(buffer->Latest(msg));

  buffer->Buffer()->Fill(std::make_shared<int>(1));
  EXPECT_TRUE(buffer->Latest(msg));
  EXPECT_EQ(1, *msg);
  EXPECT_TRUE(buffer->Latest(msg));
  EXPECT_EQ(1, *msg);

  buffer->Buffer()->Fill(std::make_shared<int>(2));
  EXPECT_TRUE(buffer->Latest(msg));
  EXPECT_EQ(2, *msg);
}

TEST(ChannelBufferTest, FetchMulti) {
  auto cache_buffer = new CacheBuffer<std::shared_ptr<int>>(2);
  auto buffer = std::make_shared<ChannelBuffer<int>>(channel0, cache_buffer);
  std::vector<std::shared_ptr<int>> vector;
  EXPECT_FALSE(buffer->FetchMulti(1, &vector));
  buffer->Buffer()->Fill(std::make_shared<int>(1));
  EXPECT_TRUE(buffer->FetchMulti(1, &vector));
  EXPECT_EQ(1, vector.size());
  EXPECT_EQ(1, *vector[0]);

  vector.clear();
  buffer->Buffer()->Fill(std::make_shared<int>(2));
  EXPECT_TRUE(buffer->FetchMulti(1, &vector));
  EXPECT_EQ(1, vector.size());
  EXPECT_EQ(2, *vector[0]);

  vector.clear();
  EXPECT_TRUE(buffer->FetchMulti(2, &vector));
  EXPECT_EQ(2, vector.size());
  EXPECT_EQ(1, *vector[0]);
  EXPECT_EQ(2, *vector[1]);

  vector.clear();
  EXPECT_TRUE(buffer->FetchMulti(3, &vector));
  EXPECT_EQ(2, vector.size());
  EXPECT_EQ(1, *vector[0]);
  EXPECT_EQ(2, *vector[1]);
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo
