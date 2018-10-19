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

#include "gtest/gtest.h"

#include "cyber/blocker/blocker_manager.h"
#include "cyber/proto/unit_test.pb.h"

namespace apollo {
namespace cyber {
namespace blocker {

using apollo::cyber::proto::UnitTest;

TEST(BlockerManagerTest, constructor) {
  BlockerManager blocker_manager;
  auto blocker = blocker_manager.GetBlocker<UnitTest>("channel");
  EXPECT_EQ(blocker, nullptr);
}

TEST(BlockerManagerTest, publish) {
  BlockerManager blocker_manager;

  auto msg1 = std::make_shared<UnitTest>();
  msg1->set_class_name("MessageTest");
  msg1->set_case_name("publish_1");

  blocker_manager.Publish<UnitTest>("channel1", msg1);
  auto blocker1 = blocker_manager.GetBlocker<UnitTest>("channel1");
  EXPECT_NE(blocker1, nullptr);
  EXPECT_FALSE(blocker1->IsPublishedEmpty());

  UnitTest msg2;
  msg2.set_class_name("MessageTest");
  msg2.set_case_name("publish_2");

  blocker_manager.Publish<UnitTest>("channel2", msg2);
  auto blocker2 = blocker_manager.GetBlocker<UnitTest>("channel2");
  EXPECT_NE(blocker2, nullptr);
  EXPECT_FALSE(blocker2->IsPublishedEmpty());

  EXPECT_TRUE(blocker1->IsObservedEmpty());
  EXPECT_TRUE(blocker2->IsObservedEmpty());

  blocker_manager.Observe();

  EXPECT_FALSE(blocker1->IsObservedEmpty());
  EXPECT_FALSE(blocker2->IsObservedEmpty());

  blocker_manager.Reset();
  auto blocker3 = blocker_manager.GetBlocker<UnitTest>("channel2");
  EXPECT_EQ(blocker3, nullptr);
}

TEST(BlockerManagerTest, subscribe) {
  BlockerManager blocker_manager;

  auto received_msg = std::make_shared<UnitTest>();
  bool res = blocker_manager.Subscribe<UnitTest>(
      "channel", 10, "BlockerManagerTest",
      [&received_msg](const std::shared_ptr<UnitTest>& blocker) {
        received_msg->CopyFrom(*blocker);
      });

  EXPECT_TRUE(res);

  auto msg1 = std::make_shared<UnitTest>();
  msg1->set_class_name("MessageTest");
  msg1->set_case_name("publish_1");

  blocker_manager.Publish<UnitTest>("channel", msg1);
  EXPECT_EQ(received_msg->class_name(), msg1->class_name());
  EXPECT_EQ(received_msg->case_name(), msg1->case_name());

  res = blocker_manager.Unsubscribe<UnitTest>("channel", "BlockerManagerTest");
  EXPECT_TRUE(res);

  res = blocker_manager.Unsubscribe<UnitTest>("channel", "BlockerManagerTest");
  EXPECT_FALSE(res);

  res =
      blocker_manager.Unsubscribe<UnitTest>("channel", "BlockerManagerTest_11");
  EXPECT_FALSE(res);

  res =
      blocker_manager.Unsubscribe<UnitTest>("channel_11", "BlockerManagerTest");
  EXPECT_FALSE(res);
}

}  // namespace blocker
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
