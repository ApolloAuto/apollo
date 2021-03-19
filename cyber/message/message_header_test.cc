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

#include "cyber/message/message_header.h"

#include "gtest/gtest.h"

namespace apollo {
namespace cyber {
namespace message {

TEST(MessageHeaderTest, magic_num) {
  MessageHeader header;
  EXPECT_FALSE(header.is_magic_num_match(nullptr, 0));
  EXPECT_FALSE(header.is_magic_num_match("ABCD", 4));
  EXPECT_TRUE(header.is_magic_num_match("BDACBDAC", 8));
  EXPECT_FALSE(header.is_magic_num_match("BDACBDACBDACBDAC", 16));
  header.reset_magic_num();
}

TEST(MessageHeaderTest, seq) {
  MessageHeader header;
  EXPECT_EQ(header.seq(), 0);
  header.set_seq(0xffffffff00000001UL);
  EXPECT_EQ(header.seq(), 0xffffffff00000001UL);
  header.reset_seq();
  EXPECT_EQ(header.seq(), 0);
}

TEST(MessageHeaderTest, timestamp_ns) {
  MessageHeader header;
  EXPECT_EQ(header.timestamp_ns(), 0);
  header.set_timestamp_ns(0xffffffff00000001UL);
  EXPECT_EQ(header.timestamp_ns(), 0xffffffff00000001UL);
  header.reset_timestamp_ns();
  EXPECT_EQ(header.timestamp_ns(), 0);
}

TEST(MessageHeaderTest, src_id) {
  MessageHeader header;
  EXPECT_EQ(header.src_id(), 0);
  header.set_src_id(0xffffffff00000001UL);
  EXPECT_EQ(header.src_id(), 0xffffffff00000001UL);
  header.reset_src_id();
  EXPECT_EQ(header.src_id(), 0);
}

TEST(MessageHeaderTest, dst_id) {
  MessageHeader header;
  EXPECT_EQ(header.dst_id(), 0);
  header.set_dst_id(0xffffffff00000001UL);
  EXPECT_EQ(header.dst_id(), 0xffffffff00000001UL);
  header.reset_dst_id();
  EXPECT_EQ(header.dst_id(), 0);
}

TEST(MessageHeaderTest, msg_type) {
  MessageHeader header;
  std::string msg_type = header.msg_type();
  EXPECT_TRUE(msg_type.empty());
  header.set_msg_type(nullptr, 1);
  msg_type = "apollo.cyber.proto.UnitTest";
  header.set_msg_type(msg_type.data(), msg_type.size());
  EXPECT_EQ(msg_type, header.msg_type());
  std::string long_type(1000, 'm');
  EXPECT_NE(long_type, header.msg_type());
}

TEST(MessageHeaderTest, content_size) {
  MessageHeader header;
  EXPECT_EQ(header.content_size(), 0);
  header.set_content_size(0xffff0001);
  EXPECT_EQ(header.content_size(), 0xffff0001);
  header.reset_content_size();
  EXPECT_EQ(header.content_size(), 0);
}

}  // namespace message
}  // namespace cyber
}  // namespace apollo
