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

#include "cyber/record/record_reader.h"

#include <string>

#include "gtest/gtest.h"

#include "cyber/record/record_writer.h"

namespace apollo {
namespace cyber {
namespace record {

using apollo::cyber::message::RawMessage;

constexpr char kChannelName1[] = "/test/channel1";
constexpr char kMessageType1[] = "apollo.cyber.proto.Test";
constexpr char kProtoDesc[] = "1234567890";
constexpr char kStr10B[] = "1234567890";
constexpr char kTestFile[] = "record_reader_test.record";
constexpr uint32_t kMessageNum = 16;

TEST(RecordTest, TestSingleRecordFile) {
  RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(kTestFile);
  writer.WriteChannel(kChannelName1, kMessageType1, kProtoDesc);
  for (uint32_t i = 0; i < kMessageNum; ++i) {
    auto msg = std::make_shared<RawMessage>(std::to_string(i));
    writer.WriteMessage(kChannelName1, msg, i);
  }
  ASSERT_EQ(kMessageNum, writer.GetMessageNumber(kChannelName1));
  ASSERT_EQ(kMessageType1, writer.GetMessageType(kChannelName1));
  ASSERT_EQ(kProtoDesc, writer.GetProtoDesc(kChannelName1));
  writer.Close();

  RecordReader reader(kTestFile);
  RecordMessage message;
  ASSERT_EQ(kMessageNum, reader.GetMessageNumber(kChannelName1));
  ASSERT_EQ(kMessageType1, reader.GetMessageType(kChannelName1));
  ASSERT_EQ(kProtoDesc, reader.GetProtoDesc(kChannelName1));

  // read all message
  uint32_t i = 0;
  for (i = 0; i < kMessageNum; ++i) {
    ASSERT_TRUE(reader.ReadMessage(&message));
    ASSERT_EQ(kChannelName1, message.channel_name);
    ASSERT_EQ(std::to_string(i), message.content);
    ASSERT_EQ(i, message.time);
  }
  ASSERT_FALSE(reader.ReadMessage(&message));

  // skip first message
  reader.Reset();
  for (i = 0; i < kMessageNum - 1; ++i) {
    ASSERT_TRUE(reader.ReadMessage(&message, 1));
    ASSERT_EQ(kChannelName1, message.channel_name);
    ASSERT_EQ(std::to_string(i + 1), message.content);
    ASSERT_EQ(i + 1, message.time);
  }
  ASSERT_FALSE(reader.ReadMessage(&message, 1));

  // skip last message
  reader.Reset();
  for (i = 0; i < kMessageNum - 1; ++i) {
    ASSERT_TRUE(reader.ReadMessage(&message, 0, kMessageNum - 2));
    ASSERT_EQ(kChannelName1, message.channel_name);
    ASSERT_EQ(std::to_string(i), message.content);
    ASSERT_EQ(i, message.time);
  }
  ASSERT_FALSE(reader.ReadMessage(&message, 0, kMessageNum - 2));
  ASSERT_FALSE(remove(kTestFile));
}

TEST(RecordTest, TestReaderOrder) {
  RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(kTestFile);
  writer.WriteChannel(kChannelName1, kMessageType1, kProtoDesc);

  for (uint32_t i = kMessageNum; i > 0; --i) {
    auto msg = std::make_shared<RawMessage>(std::to_string(i));
    writer.WriteMessage(kChannelName1, msg, i * 100);
  }
  ASSERT_EQ(kMessageNum, writer.GetMessageNumber(kChannelName1));
  ASSERT_EQ(kMessageType1, writer.GetMessageType(kChannelName1));
  ASSERT_EQ(kProtoDesc, writer.GetProtoDesc(kChannelName1));
  writer.Close();

  RecordReader reader(kTestFile);
  RecordMessage message;
  ASSERT_EQ(kMessageNum, reader.GetMessageNumber(kChannelName1));
  ASSERT_EQ(kMessageType1, reader.GetMessageType(kChannelName1));
  ASSERT_EQ(kProtoDesc, reader.GetProtoDesc(kChannelName1));

  // read all message
  for (uint32_t i = 1; i <= kMessageNum; ++i) {
    ASSERT_TRUE(reader.ReadMessage(&message));
    ASSERT_EQ(kChannelName1, message.channel_name);
    ASSERT_NE(std::to_string(i), message.content);
    ASSERT_NE(i * 100, message.time);
  }

  ASSERT_FALSE(reader.ReadMessage(&message));
  ASSERT_FALSE(remove(kTestFile));
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
