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

#include "cyber/message/message_traits.h"
#include "cyber/proto/unit_test.pb.h"

namespace apollo {
namespace cyber {
namespace message {

TEST(MessageTraitsTest, serialize_to_string) {
  std::string str("");
  proto::UnitTest ut;
  ut.set_class_name("MessageTraits");
  ut.set_case_name("serialize_to_string");

  EXPECT_TRUE(SerializeToString(ut, &str));
  EXPECT_EQ(str, "\n\rMessageTraits\x12\x13serialize_to_string");
}

TEST(MessageTraitsTest, parse_from_string) {
  proto::UnitTest ut;
  std::string str("\n\rMessageTraits\x12\x11parse_from_string");
  EXPECT_TRUE(ParseFromString(str, &ut));
  EXPECT_EQ(ut.class_name(), "MessageTraits");
  EXPECT_EQ(ut.case_name(), "parse_from_string");
}

TEST(MessageTraitsTest, message_type) {
  std::string msg_type = MessageType<proto::UnitTest>();
  EXPECT_EQ(msg_type, "apollo.cyber.proto.UnitTest");

  proto::UnitTest ut;
  msg_type = MessageType(ut);
  EXPECT_EQ(msg_type, "apollo.cyber.proto.UnitTest");
}

}  // namespace message
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
