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

#include "cybertron/dispatcher/message.h"
#include "cybertron/proto/unit_test.pb.h"

namespace apollo {
namespace cybertron {
namespace dispatcher {

using apollo::cybertron::proto::UnitTest;

TEST(MessageTest, constructor) {
  MessageAttr attr(10, "channel");
  Message<UnitTest> message(attr);
  EXPECT_EQ(message.capacity(), 10);
  EXPECT_EQ(message.channel_name(), "channel");

  message.set_capacity(20);
  EXPECT_EQ(message.capacity(), 20);
}

TEST(MessageTest, publish) {
  MessageAttr attr(10, "channel");
  Message<UnitTest> message(attr);

  auto msg1 = std::make_shared<UnitTest>();
  msg1->set_class_name("MessageTest");
  msg1->set_case_name("publish_1");

  UnitTest msg2;
  msg2.set_class_name("MessageTest");
  msg2.set_case_name("publish_2");

  EXPECT_TRUE(message.IsPublishedEmpty());
  message.Publish(msg1);
  message.Publish(msg2);
  EXPECT_FALSE(message.IsPublishedEmpty());

  EXPECT_TRUE(message.IsObservedEmpty());
  message.Observe();
  EXPECT_FALSE(message.IsObservedEmpty());

  auto& latest_observed_msg = message.GetLatestObserved();
  EXPECT_EQ(latest_observed_msg.class_name(), "MessageTest");
  EXPECT_EQ(latest_observed_msg.case_name(), "publish_2");

  auto latest_observed_msg_ptr = message.GetLatestObservedPtr();
  EXPECT_EQ(latest_observed_msg_ptr->class_name(), "MessageTest");
  EXPECT_EQ(latest_observed_msg_ptr->case_name(), "publish_2");

  auto latest_published_ptr = message.GetLatestPublishedPtr();
  EXPECT_EQ(latest_published_ptr->class_name(), "MessageTest");
  EXPECT_EQ(latest_published_ptr->case_name(), "publish_2");

  message.ClearPublished();
  EXPECT_TRUE(message.IsPublishedEmpty());
  EXPECT_FALSE(message.IsObservedEmpty());
}

TEST(MessageTest, subscribe) {
  MessageAttr attr(10, "channel");
  Message<UnitTest> message(attr);

  auto received_msg = std::make_shared<UnitTest>();
  bool res = message.Subscribe(
      "MessageTest1", [&received_msg](const std::shared_ptr<UnitTest>& msg) {
        received_msg->CopyFrom(*msg);
      });

  EXPECT_TRUE(res);

  auto msg1 = std::make_shared<UnitTest>();
  msg1->set_class_name("MessageTest");
  msg1->set_case_name("publish_1");

  message.Publish(msg1);

  EXPECT_EQ(received_msg->class_name(), msg1->class_name());
  EXPECT_EQ(received_msg->case_name(), msg1->case_name());

  res = message.Subscribe(
      "MessageTest1", [&received_msg](const std::shared_ptr<UnitTest>& msg) {
        received_msg->CopyFrom(*msg);
      });

  EXPECT_FALSE(res);

  res = message.Unsubscribe("MessageTest1");
  EXPECT_TRUE(res);
  res = message.Unsubscribe("MessageTest1");
  EXPECT_FALSE(res);

  UnitTest msg2;
  msg2.set_class_name("MessageTest");
  msg2.set_case_name("publish_2");

  message.Publish(msg2);
  EXPECT_NE(received_msg->case_name(), msg2.case_name());
}

}  // namespace dispatcher
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
