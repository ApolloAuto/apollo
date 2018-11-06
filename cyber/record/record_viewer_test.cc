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
#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <memory>
#include <string>

#include "cyber/common/log.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_viewer.h"
#include "cyber/record/record_writer.h"

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
const uint32_t MESSAGE_NUM = 16;

void ConstructRecord(uint32_t begin_time, uint32_t end_time) {
  RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(TEST_FILE);
  writer.WriteChannel(CHANNEL_NAME_1, MESSAGE_TYPE_1, PROTO_DESC);
  for (unsigned int i = begin_time; i <= end_time; ++i) {
    auto msg = std::make_shared<RawMessage>(std::to_string(i));
    writer.WriteMessage(CHANNEL_NAME_1, msg, i);
  }
  ASSERT_EQ(end_time - begin_time + 1, writer.GetMessageNumber(CHANNEL_NAME_1));
  writer.Close();
}

uint64_t CheckCount(RecordViewer viewer) {
  int count = 0;
  for (auto& msg : viewer) {
    if (msg.time >= 0) {
      count++;
    }
  }
  return count;
}

TEST(RecordTest, iterator_test) {
  ConstructRecord(1, 16);
  auto reader = std::make_shared<RecordReader>(TEST_FILE);
  RecordViewer viewer(reader);
  EXPECT_TRUE(viewer.IsValid());
  EXPECT_EQ(1, viewer.begin_time());
  EXPECT_EQ(16, viewer.end_time());

  int count = 0;
  for (auto& msg : viewer) {
    count++;
    EXPECT_EQ(CHANNEL_NAME_1, msg.channel_name);
    EXPECT_EQ(count, msg.time);
    EXPECT_EQ(std::to_string(count), msg.content);
  }
  EXPECT_EQ(MESSAGE_NUM, count);

  count = 0;
  std::for_each(viewer.begin(), viewer.end(), [&count](RecordMessage& msg) {
    count++;
    EXPECT_EQ(CHANNEL_NAME_1, msg.channel_name);
    EXPECT_EQ(count, msg.time);
    EXPECT_EQ(std::to_string(count), msg.content);
  });
  EXPECT_EQ(MESSAGE_NUM, count);

  count = 0;
  for (auto it = viewer.begin(); it != viewer.end(); ++it) {
    count++;
    EXPECT_EQ(CHANNEL_NAME_1, it->channel_name);
    EXPECT_EQ(count, it->time);
    EXPECT_EQ(std::to_string(count), it->content);
  }
  EXPECT_EQ(MESSAGE_NUM, count);
}

TEST(RecordTest, filter_test) {
  int START_TIME = 1;
  int END_TIME = 1000;
  ConstructRecord(START_TIME, END_TIME);

  auto reader = std::make_shared<RecordReader>(TEST_FILE);
  RecordViewer viewer_0(reader);
  EXPECT_EQ(CheckCount(viewer_0), 1000);
  EXPECT_EQ(START_TIME, viewer_0.begin_time());
  EXPECT_EQ(END_TIME, viewer_0.end_time());

  RecordViewer viewer_1(reader, END_TIME, END_TIME);
  EXPECT_EQ(CheckCount(viewer_1), 1);
  EXPECT_EQ(END_TIME, viewer_1.begin_time());

  RecordViewer viewer_2(reader, END_TIME, START_TIME);
  EXPECT_EQ(CheckCount(viewer_2), 0);
  EXPECT_EQ(START_TIME, viewer_2.end_time());

  viewer_0.begin();
  viewer_0.begin();

  RecordViewer viewer_3(reader, 1, END_TIME);
  EXPECT_EQ(CheckCount(viewer_3), 1000);

  RecordViewer viewer_4(reader, START_TIME);
  EXPECT_EQ(CheckCount(viewer_4), 1000);

  auto it_1 = viewer_3.begin();
  auto it_2 = viewer_4.begin();
  EXPECT_FALSE(it_1 == it_2);

  RecordViewer viewer_5(reader, 500, 600);
  EXPECT_EQ(CheckCount(viewer_5), 101);

  RecordViewer viewer_6(reader, 0, END_TIME, {"null"});
  EXPECT_EQ(CheckCount(viewer_6), 0);

  RecordViewer viewer_7(reader, 0, END_TIME, {CHANNEL_NAME_1});
  EXPECT_EQ(CheckCount(viewer_7), 1000);
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
