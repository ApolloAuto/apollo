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
#include "cyber/record/record_writer.h"

#include <gtest/gtest.h>
#include <string>

namespace apollo {
namespace cyber {
namespace record {

using apollo::cyber::message::RawMessage;

const char CHANNEL_NAME_1[] = "/test/channel1";
const char CHANNEL_NAME_2[] = "/test/channel2";
const char MESSAGE_TYPE_1[] = "apollo.cyber.proto.Test";
const char MESSAGE_TYPE_2[] = "apollo.cyber.proto.Channel";
const char PROTO_DESC[] = "1234567890";
const char STR_10B[] = "1234567890";
const char TEST_FILE[] = "test.record";
const uint64_t TIME_1 = 1000 * 1e6;
const uint64_t TIME_2 = 1010 * 1e6;
const uint64_t TIME_3 = 1020 * 1e6;
const uint32_t MESSAGE_NUM = 16;

TEST(RecordTest, TestSingleRecordFile) {
  RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(TEST_FILE);
  writer.WriteChannel(CHANNEL_NAME_1, MESSAGE_TYPE_1, PROTO_DESC);
  for (uint32_t i = 0; i < MESSAGE_NUM; ++i) {
    auto msg = std::make_shared<RawMessage>(std::to_string(i));
    writer.WriteMessage(CHANNEL_NAME_1, msg, i);
  }
  ASSERT_EQ(MESSAGE_NUM, writer.GetMessageNumber(CHANNEL_NAME_1));
  ASSERT_EQ(MESSAGE_TYPE_1, writer.GetMessageType(CHANNEL_NAME_1));
  ASSERT_EQ(PROTO_DESC, writer.GetProtoDesc(CHANNEL_NAME_1));
  writer.Close();

  RecordReader reader(TEST_FILE);
  RecordMessage message;
  ASSERT_EQ(MESSAGE_NUM, reader.GetMessageNumber(CHANNEL_NAME_1));
  ASSERT_EQ(MESSAGE_TYPE_1, reader.GetMessageType(CHANNEL_NAME_1));
  ASSERT_EQ(PROTO_DESC, reader.GetProtoDesc(CHANNEL_NAME_1));

  // read all message
  uint32_t i = 0;
  for (i = 0; i < MESSAGE_NUM; ++i) {
    ASSERT_TRUE(reader.ReadMessage(&message));
    ASSERT_EQ(CHANNEL_NAME_1, message.channel_name);
    ASSERT_EQ(std::to_string(i), message.content);
    ASSERT_EQ(i, message.time);
  }
  ASSERT_FALSE(reader.ReadMessage(&message));

  // skip first message
  reader.Reset();
  for (i = 0; i < MESSAGE_NUM - 1; ++i) {
    ASSERT_TRUE(reader.ReadMessage(&message, 1));
    ASSERT_EQ(CHANNEL_NAME_1, message.channel_name);
    ASSERT_EQ(std::to_string(i + 1), message.content);
    ASSERT_EQ(i + 1, message.time);
  }
  ASSERT_FALSE(reader.ReadMessage(&message, 1));

  // skip last message
  reader.Reset();
  for (i = 0; i < MESSAGE_NUM - 1; ++i) {
    ASSERT_TRUE(reader.ReadMessage(&message, 0, MESSAGE_NUM - 2));
    ASSERT_EQ(CHANNEL_NAME_1, message.channel_name);
    ASSERT_EQ(std::to_string(i), message.content);
    ASSERT_EQ(i, message.time);
  }
  ASSERT_FALSE(reader.ReadMessage(&message, 0, MESSAGE_NUM - 2));
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
