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

#include "cybertron/dispatcher/dispatcher.h"
#include "cybertron/proto/unit_test.pb.h"

namespace apollo {
namespace cybertron {
namespace dispatcher {

using apollo::cybertron::proto::UnitTest;

TEST(DispatcherTest, constructor) {
  Dispatcher dispatcher;
  auto msg = dispatcher.GetMessage<UnitTest>("channel");
  EXPECT_EQ(msg, nullptr);
}

TEST(DispatcherTest, publish) {
  Dispatcher dispatcher;

  auto msg1 = std::make_shared<UnitTest>();
  msg1->set_class_name("MessageTest");
  msg1->set_case_name("publish_1");

  dispatcher.Publish<UnitTest>("channel1", msg1);
  auto message1 = dispatcher.GetMessage<UnitTest>("channel1");
  EXPECT_NE(message1, nullptr);
  EXPECT_FALSE(message1->IsPublishedEmpty());

  UnitTest msg2;
  msg2.set_class_name("MessageTest");
  msg2.set_case_name("publish_2");

  dispatcher.Publish<UnitTest>("channel2", msg2);
  auto message2 = dispatcher.GetMessage<UnitTest>("channel2");
  EXPECT_NE(message2, nullptr);
  EXPECT_FALSE(message2->IsPublishedEmpty());

  EXPECT_TRUE(message1->IsObservedEmpty());
  EXPECT_TRUE(message2->IsObservedEmpty());

  dispatcher.Observe();

  EXPECT_FALSE(message1->IsObservedEmpty());
  EXPECT_FALSE(message2->IsObservedEmpty());
}

TEST(DispatcherTest, subscribe) {
  Dispatcher dispatcher;

  auto received_msg = std::make_shared<UnitTest>();
  bool res = dispatcher.Subscribe<UnitTest>(
      "channel", 10, "DispatcherTest",
      [&received_msg](const std::shared_ptr<UnitTest>& msg) {
        received_msg->CopyFrom(*msg);
      });

  EXPECT_TRUE(res);

  auto msg1 = std::make_shared<UnitTest>();
  msg1->set_class_name("MessageTest");
  msg1->set_case_name("publish_1");

  dispatcher.Publish<UnitTest>("channel", msg1);
  EXPECT_EQ(received_msg->class_name(), msg1->class_name());
  EXPECT_EQ(received_msg->case_name(), msg1->case_name());

  res = dispatcher.Unsubscribe<UnitTest>("channel", "DispatcherTest");
  EXPECT_TRUE(res);

  res = dispatcher.Unsubscribe<UnitTest>("channel", "DispatcherTest");
  EXPECT_FALSE(res);

  res = dispatcher.Unsubscribe<UnitTest>("channel", "DispatcherTest_11");
  EXPECT_FALSE(res);

  res = dispatcher.Unsubscribe<UnitTest>("channel_11", "DispatcherTest");
  EXPECT_FALSE(res);
}

}  // namespace dispatcher
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
