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

#include "cyber/message/message_traits.h"

#include <string>

#include "gtest/gtest.h"

#include "cyber/proto/unit_test.pb.h"

namespace apollo {
namespace cyber {
namespace message {

class Data {
  uint64_t timestamp;
  std::string content;
};

class Message {
 public:
  std::string content;

  std::size_t ByteSizeLong() const { return content.size(); }

  bool SerializeToArray(void* data, int size) const {
    if (data == nullptr || size < 0 ||
        static_cast<size_t>(size) < ByteSizeLong()) {
      return false;
    }

    memcpy(data, content.data(), content.size());
    return true;
  }

  bool SerializeToString(std::string* str) const {
    *str = content;
    return true;
  }

  bool ParseFromArray(const void* data, int size) {
    if (data == nullptr || size <= 0) {
      return false;
    }
    content.assign(static_cast<const char*>(data), size);
    return true;
  }

  bool ParseFromString(const std::string& str) {
    content = str;
    return true;
  }

  static void GetDescriptorString(const std::string&, std::string* str) {
    *str = "message";
  }

  std::string TypeName() const { return "type"; }
};

class PbMessage {
 public:
  static std::string TypeName() { return "protobuf"; }
};

TEST(MessageTraitsTest, type_trait) {
  EXPECT_FALSE(HasType<Data>::value);
  EXPECT_FALSE(HasSerializer<Data>::value);
  EXPECT_FALSE(HasGetDescriptorString<Data>::value);

  EXPECT_TRUE(HasType<Message>::value);
  EXPECT_TRUE(HasSerializer<Message>::value);
  EXPECT_TRUE(HasGetDescriptorString<Message>::value);

  EXPECT_TRUE(HasSerializer<proto::UnitTest>::value);

  EXPECT_TRUE(HasType<PyMessageWrap>::value);
  EXPECT_TRUE(HasSerializer<PyMessageWrap>::value);
  EXPECT_TRUE(HasGetDescriptorString<PyMessageWrap>::value);

  EXPECT_TRUE(HasType<RawMessage>::value);
  EXPECT_TRUE(HasSerializer<RawMessage>::value);
  EXPECT_TRUE(HasGetDescriptorString<RawMessage>::value);

  Message msg;
  EXPECT_EQ("type", MessageType<Message>(msg));

  PbMessage pb_msg;
  EXPECT_EQ("protobuf", MessageType<PbMessage>(pb_msg));
  EXPECT_EQ("protobuf", MessageType<PbMessage>());
}

TEST(MessageTraitsTest, byte_size) {
  Data data;
  EXPECT_EQ(ByteSize(data), -1);

  Message msg;
  EXPECT_EQ(ByteSize(msg), 0);
  msg.content = "123";
  EXPECT_EQ(ByteSize(msg), 3);

  proto::UnitTest ut;
  ut.set_class_name("MessageTraitsTest");
  ut.set_case_name("byte_size");
  EXPECT_GT(ByteSize(ut), 0);

  RawMessage raw_msg;
  EXPECT_EQ(ByteSize(raw_msg), 0);
  raw_msg.message = "123";
  EXPECT_EQ(ByteSize(raw_msg), 3);

  PyMessageWrap py_msg;
  EXPECT_EQ(ByteSize(py_msg), 0);
  py_msg.set_data("123");
  EXPECT_EQ(ByteSize(py_msg), 3);
}

TEST(MessageTraitsTest, serialize_to_array) {
  const int kArraySize = 256;
  char array[kArraySize] = {0};

  proto::UnitTest ut;
  ut.set_class_name("MessageTraits");
  ut.set_case_name("serialize_to_string");
  EXPECT_TRUE(SerializeToArray(ut, array, sizeof(array)));
  {
    std::string arr_str(array);
    EXPECT_EQ(arr_str, "\n\rMessageTraits\x12\x13serialize_to_string");
  }

  memset(array, 0, sizeof(array));
  Data data;
  EXPECT_FALSE(SerializeToArray(data, array, sizeof(array)));
  EXPECT_EQ(strlen(array), 0);

  memset(array, 0, sizeof(array));
  Message msg{"content"};
  EXPECT_TRUE(SerializeToArray(msg, array, sizeof(array)));
  {
    std::string arr_str(array);
    EXPECT_EQ("content", arr_str);
  }

  memset(array, 0, sizeof(array));
  RawMessage raw("content");
  EXPECT_TRUE(SerializeToArray(raw, array, sizeof(array)));
  {
    std::string arr_str(array);
    EXPECT_EQ("content", arr_str);
  }
}

TEST(MessageTraitsTest, serialize_to_string) {
  std::string str("");

  // protobuf message
  proto::UnitTest ut;
  ut.set_class_name("MessageTraits");
  ut.set_case_name("serialize_to_string");
  EXPECT_TRUE(SerializeToString(ut, &str));
  EXPECT_EQ(str, "\n\rMessageTraits\x12\x13serialize_to_string");

  str = "";
  Data data;
  EXPECT_FALSE(SerializeToString(data, &str));
  EXPECT_EQ("", str);

  str = "";
  Message msg{"content"};
  EXPECT_TRUE(SerializeToString(msg, &str));
  EXPECT_EQ("content", str);

  str = "";
  RawMessage raw("content");
  EXPECT_TRUE(SerializeToString(raw, &str));
  EXPECT_EQ("content", str);
}

TEST(MessageTraitsTest, parse_from_array) {
  const int kArraySize = 256;
  const char array[kArraySize] = "\n\rMessageTraits\x12\x11parse_from_string";
  const int arr_str_len = static_cast<int>(strlen(array));
  const std::string arr_str(array);

  proto::UnitTest ut;
  EXPECT_TRUE(ParseFromArray(array, arr_str_len, &ut));
  EXPECT_EQ(ut.class_name(), "MessageTraits");
  EXPECT_EQ(ut.case_name(), "parse_from_string");

  Data data;
  EXPECT_FALSE(ParseFromArray(array, arr_str_len, &data));

  Message msg{"content"};
  EXPECT_TRUE(ParseFromArray(array, arr_str_len, &msg));
  EXPECT_EQ(msg.content, arr_str);

  RawMessage raw;
  EXPECT_TRUE(ParseFromArray(array, arr_str_len, &raw));
  EXPECT_EQ(raw.message, arr_str);
}

TEST(MessageTraitsTest, parse_from_string) {
  proto::UnitTest ut;
  std::string str("\n\rMessageTraits\x12\x11parse_from_string");
  EXPECT_TRUE(ParseFromString(str, &ut));
  EXPECT_EQ(ut.class_name(), "MessageTraits");
  EXPECT_EQ(ut.case_name(), "parse_from_string");

  Data data;
  EXPECT_FALSE(ParseFromString(str, &data));

  Message msg{"content"};
  EXPECT_TRUE(ParseFromString(str, &msg));
  EXPECT_EQ("\n\rMessageTraits\x12\x11parse_from_string", msg.content);

  RawMessage raw;
  EXPECT_TRUE(ParseFromString(str, &raw));
  EXPECT_EQ(str, raw.message);
}

TEST(MessageTraitsTest, serialize_parse_hc) {
  auto msg = std::make_shared<proto::Chatter>();
  msg->set_timestamp(12345);
  msg->set_seq(1);
  msg->set_content("chatter msg");

  const int size = ByteSize(*msg) + static_cast<int>(sizeof(MessageHeader));
  std::string buffer;
  buffer.resize(size);
  EXPECT_TRUE(SerializeToHC(*msg, const_cast<char*>(buffer.data()), size));

  auto pb_msg = std::make_shared<proto::Chatter>();
  auto raw_msg = std::make_shared<RawMessage>();

  EXPECT_TRUE(
      ParseFromHC(const_cast<char*>(buffer.data()), size, pb_msg.get()));
  EXPECT_TRUE(
      ParseFromHC(const_cast<char*>(buffer.data()), size, raw_msg.get()));

  std::string new_buffer;
  new_buffer.resize(size);
  EXPECT_TRUE(
      SerializeToHC(*pb_msg, const_cast<char*>(new_buffer.data()), size));
  EXPECT_EQ(new_buffer, buffer);
  new_buffer.clear();

  new_buffer.resize(size);
  EXPECT_TRUE(
      SerializeToHC(*raw_msg, const_cast<char*>(new_buffer.data()), size));
  EXPECT_TRUE(
      ParseFromHC(const_cast<char*>(new_buffer.data()), size, pb_msg.get()));
  EXPECT_EQ(pb_msg->timestamp(), 12345);
  EXPECT_EQ(pb_msg->seq(), 1);
  EXPECT_EQ(pb_msg->content(), "chatter msg");
}

TEST(MessageTraitsTest, message_type) {
  std::string msg_type = MessageType<proto::UnitTest>();
  EXPECT_EQ(msg_type, "apollo.cyber.proto.UnitTest");

  proto::UnitTest ut;
  msg_type = MessageType(ut);
  EXPECT_EQ(msg_type, "apollo.cyber.proto.UnitTest");
}

TEST(MessageTraitsTest, descriptor) {
  const std::string pb_desc =
      "\n\xFA\x1\n\x1B"
      "cyber/proto/unit_test.proto\x12\x12"
      "apollo.cyber.proto\"1\n\bUnitTest\x12\x12\n\nclass_name\x18\x1 "
      "\x1(\t\x12\x11\n\tcase_name\x18\x2 "
      "\x1(\t\"S\n\aChatter\x12\x11\n\ttimestamp\x18\x1 "
      "\x1(\x4\x12\x17\n\xFlidar_timestamp\x18\x2 \x1(\x4\x12\v\n\x3seq\x18\x3 "
      "\x1(\x4\x12\xF\n\acontent\x18\x4 \x1(\f\"?\n\x10"
      "ChatterBenchmark\x12\r\n\x5stamp\x18\x1 \x1(\x4\x12\v\n\x3seq\x18\x2 "
      "\x1(\x4\x12\xF\n\acontent\x18\x3 \x1(\t";
  std::string desc;
  GetDescriptorString<proto::UnitTest>("apollo.cyber.proto.UnitTest", &desc);
  EXPECT_EQ(pb_desc, desc);

  desc = "";
  GetDescriptorString<RawMessage>("apollo.cyber.proto.UnitTest", &desc);
  EXPECT_EQ(pb_desc, desc);

  desc = "";
  GetDescriptorString<Data>("apollo", &desc);
  EXPECT_EQ("", desc);

  desc = "";
  GetDescriptorString<Message>("apollo", &desc);
  EXPECT_EQ("message", desc);
}

}  // namespace message
}  // namespace cyber
}  // namespace apollo
