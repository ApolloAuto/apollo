/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>
#include <vector>
#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/data/data_visitor.h"
#include "cyber/data/fusion/all_latest.h"

namespace apollo {
namespace cyber {
namespace data {

using apollo::cyber::message::RawMessage;
using apollo::cyber::proto::RoleAttributes;
std::hash<std::string> str_hash;

TEST(AllLatestTest, two_channels) {
  auto cache0 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  auto cache1 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  ChannelBuffer<RawMessage> buffer0(static_cast<uint64_t>(0), cache0);
  ChannelBuffer<RawMessage> buffer1(static_cast<uint64_t>(1), cache1);
  std::shared_ptr<RawMessage> m;
  std::shared_ptr<RawMessage> m0;
  std::shared_ptr<RawMessage> m1;
  uint64_t index = 0;
  fusion::AllLatest<RawMessage, RawMessage> fusion(buffer0, buffer1);

  // normal fusion
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1));
  cache0->Fill(std::make_shared<RawMessage>("0-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1));
  cache1->Fill(std::make_shared<RawMessage>("1-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1));
  cache0->Fill(std::make_shared<RawMessage>("0-1"));
  EXPECT_TRUE(fusion.Fusion(&index, m0, m1));
  index++;
  EXPECT_EQ(std::string("0-1"), m0->message);
  EXPECT_EQ(std::string("1-0"), m1->message);
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1));

  // use 0-2 and lateste m1
  cache0->Fill(std::make_shared<RawMessage>("0-2"));
  EXPECT_TRUE(fusion.Fusion(&index, m0, m1));
  index++;
  EXPECT_EQ(std::string("0-2"), m0->message);
  EXPECT_EQ(std::string("1-0"), m1->message);
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1));

  // sub channel will not trigger fusion
  cache1->Fill(std::make_shared<RawMessage>("1-1"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1));

  // m0 overflow
  for (int i = 0; i < 100; i++) {
    cache0->Fill(std::make_shared<RawMessage>(std::string("0-") +
                                              std::to_string(2 + i + 1)));
  }
  // EXPECT_TRUE(fusion.buffer_fusion_->Buffer()->Full());
  EXPECT_TRUE(fusion.Fusion(&index, m0, m1));
  index++;
  EXPECT_EQ(std::string("0-102"), m0->message);
}

TEST(AllLatestTest, three_channels) {
  auto cache0 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  auto cache1 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  auto cache2 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  ChannelBuffer<RawMessage> buffer0(0, cache0);
  ChannelBuffer<RawMessage> buffer1(1, cache1);
  ChannelBuffer<RawMessage> buffer2(2, cache2);
  std::shared_ptr<RawMessage> m;
  std::shared_ptr<RawMessage> m0;
  std::shared_ptr<RawMessage> m1;
  std::shared_ptr<RawMessage> m2;
  uint64_t index = 0;
  fusion::AllLatest<RawMessage, RawMessage, RawMessage> fusion(buffer0, buffer1,
                                                               buffer2);

  // normal fusion
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2));
  cache0->Fill(std::make_shared<RawMessage>("0-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2));
  cache1->Fill(std::make_shared<RawMessage>("1-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2));
  cache2->Fill(std::make_shared<RawMessage>("2-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2));
  cache0->Fill(std::make_shared<RawMessage>("0-1"));
  EXPECT_TRUE(fusion.Fusion(&index, m0, m1, m2));
  index++;
  EXPECT_EQ(std::string("0-1"), m0->message);
  EXPECT_EQ(std::string("1-0"), m1->message);
  EXPECT_EQ(std::string("2-0"), m2->message);
}

TEST(AllLatestTest, four_channels) {
  auto cache0 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  auto cache1 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  auto cache2 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  auto cache3 = new CacheBuffer<std::shared_ptr<RawMessage>>(10);
  ChannelBuffer<RawMessage> buffer0(0, cache0);
  ChannelBuffer<RawMessage> buffer1(1, cache1);
  ChannelBuffer<RawMessage> buffer2(2, cache2);
  ChannelBuffer<RawMessage> buffer3(3, cache3);
  std::shared_ptr<RawMessage> m;
  std::shared_ptr<RawMessage> m0;
  std::shared_ptr<RawMessage> m1;
  std::shared_ptr<RawMessage> m2;
  std::shared_ptr<RawMessage> m3;
  uint64_t index = 0;
  fusion::AllLatest<RawMessage, RawMessage, RawMessage, RawMessage> fusion(
      buffer0, buffer1, buffer2, buffer3);

  // normal fusion
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2, m3));
  cache0->Fill(std::make_shared<RawMessage>("0-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2, m3));
  cache1->Fill(std::make_shared<RawMessage>("1-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2, m3));
  cache2->Fill(std::make_shared<RawMessage>("2-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2, m3));
  cache3->Fill(std::make_shared<RawMessage>("3-0"));
  EXPECT_FALSE(fusion.Fusion(&index, m0, m1, m2, m3));
  cache0->Fill(std::make_shared<RawMessage>("0-1"));
  EXPECT_TRUE(fusion.Fusion(&index, m0, m1, m2, m3));
  index++;
  EXPECT_EQ(std::string("0-1"), m0->message);
  EXPECT_EQ(std::string("1-0"), m1->message);
  EXPECT_EQ(std::string("2-0"), m2->message);
  EXPECT_EQ(std::string("3-0"), m3->message);
}

}  // namespace data
}  // namespace cyber
}  // namespace apollo
