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

#include <gtest/gtest.h>
#include <string>

#include "cyber/message/raw_message.h"

namespace apollo {
namespace cyber {
namespace message {

TEST(RawMessageTest, constructor) {
  RawMessage msg_a;
  EXPECT_EQ(msg_a.message, "");

  RawMessage msg_b("raw");
  EXPECT_EQ(msg_b.message, "raw");
}

TEST(RawMessageTest, serialize_to_string) {
  RawMessage msg("serialize_to_string");
  std::string str("");
  EXPECT_FALSE(msg.SerializeToString(nullptr));
  EXPECT_TRUE(msg.SerializeToString(&str));
  EXPECT_EQ(str, "serialize_to_string");
}

TEST(RawMessageTest, parse_from_string) {
  RawMessage msg;
  EXPECT_TRUE(msg.ParseFromString("parse_from_string"));
  EXPECT_EQ(msg.message, "parse_from_string");
}

TEST(RawMessageTest, message_type) {
  RawMessage msg;
  std::string msg_type = RawMessage::TypeName();
  EXPECT_EQ(msg_type, "apollo.cyber.message.RawMessage");

  // msg_type = MessageType<RawMessage>();
  // EXPECT_EQ(msg_type, "apollo.cyber.message.RawMessage");
}

}  // namespace message
}  // namespace cyber
}  // namespace apollo
