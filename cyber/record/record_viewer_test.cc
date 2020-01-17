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

#include "cyber/record/record_viewer.h"

#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

namespace apollo {
namespace cyber {
namespace record {

using apollo::cyber::message::RawMessage;

constexpr char kChannelName1[] = "/test/channel1";
constexpr char kMessageType1[] = "apollo.cyber.proto.Test";
constexpr char kProtoDesc1[] = "1234567890";
constexpr char kTestFile[] = "viewer_test.record";

void ConstructRecord(uint64_t msg_num, uint64_t begin_time,
                     uint64_t time_step) {
  RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(kTestFile);
  writer.WriteChannel(kChannelName1, kMessageType1, kProtoDesc1);
  for (uint64_t i = 0; i < msg_num; i++) {
    auto msg = std::make_shared<RawMessage>(std::to_string(i));
    writer.WriteMessage(kChannelName1, msg, begin_time + time_step * i);
  }
  ASSERT_EQ(msg_num, writer.GetMessageNumber(kChannelName1));
  writer.Close();
}

uint64_t CheckCount(RecordViewer viewer) {
  int i = 0;
  for (auto& msg : viewer) {
    if (msg.time >= 0) {
      i++;
    }
  }
  return i;
}

TEST(RecordTest, iterator_test) {
  uint64_t msg_num = 200;
  uint64_t begin_time = 100000000;
  uint64_t step_time = 100000000;  // 100ms
  uint64_t end_time = begin_time + step_time * (msg_num - 1);
  ConstructRecord(msg_num, begin_time, step_time);

  auto reader = std::make_shared<RecordReader>(kTestFile);
  RecordViewer viewer(reader);
  EXPECT_TRUE(viewer.IsValid());
  EXPECT_EQ(begin_time, viewer.begin_time());
  EXPECT_EQ(end_time, viewer.end_time());

  uint64_t i = 0;
  for (auto& msg : viewer) {
    EXPECT_EQ(kChannelName1, msg.channel_name);
    EXPECT_EQ(begin_time + step_time * i, msg.time);
    EXPECT_EQ(std::to_string(i), msg.content);
    i++;
  }
  EXPECT_EQ(msg_num, i);

  i = 0;
  std::for_each(viewer.begin(), viewer.end(), [&i](RecordMessage& msg) {
    EXPECT_EQ(kChannelName1, msg.channel_name);
    // EXPECT_EQ(begin_time + step_time * i, msg.time);
    EXPECT_EQ(std::to_string(i), msg.content);
    i++;
  });
  EXPECT_EQ(msg_num, i);

  i = 0;
  for (auto it = viewer.begin(); it != viewer.end(); ++it) {
    EXPECT_EQ(kChannelName1, it->channel_name);
    EXPECT_EQ(begin_time + step_time * i, it->time);
    EXPECT_EQ(std::to_string(i), it->content);
    i++;
  }
  EXPECT_EQ(msg_num, i);
  ASSERT_FALSE(remove(kTestFile));
}

TEST(RecordTest, filter_test) {
  uint64_t msg_num = 200;
  uint64_t begin_time = 100000000;
  uint64_t step_time = 100000000;  // 100ms
  uint64_t end_time = begin_time + step_time * (msg_num - 1);
  ConstructRecord(msg_num, begin_time, step_time);

  auto reader = std::make_shared<RecordReader>(kTestFile);
  RecordViewer viewer_0(reader);
  EXPECT_EQ(CheckCount(viewer_0), msg_num);
  EXPECT_EQ(begin_time, viewer_0.begin_time());
  EXPECT_EQ(end_time, viewer_0.end_time());

  RecordViewer viewer_1(reader, end_time, end_time);
  EXPECT_EQ(CheckCount(viewer_1), 1);
  EXPECT_EQ(end_time, viewer_1.begin_time());

  RecordViewer viewer_2(reader, end_time, begin_time);
  EXPECT_EQ(CheckCount(viewer_2), 0);
  EXPECT_EQ(begin_time, viewer_2.end_time());

  viewer_0.begin();
  viewer_0.begin();

  // filter first message
  RecordViewer viewer_3(reader, begin_time + 1, end_time);
  EXPECT_EQ(CheckCount(viewer_3), msg_num - 1);

  // filter last message
  RecordViewer viewer_4(reader, begin_time, end_time - 1);
  EXPECT_EQ(CheckCount(viewer_4), msg_num - 1);

  auto it_1 = viewer_3.begin();
  auto it_2 = viewer_4.begin();
  EXPECT_NE(it_1, it_2);

  // pick 2 frame
  RecordViewer viewer_5(reader, begin_time + 12 * step_time,
                        begin_time + 13 * step_time);
  EXPECT_EQ(CheckCount(viewer_5), 2);

  // filter with not exist channel
  RecordViewer viewer_6(reader, 0, end_time, {"null"});
  EXPECT_EQ(CheckCount(viewer_6), 0);

  // filter with exist channel
  RecordViewer viewer_7(reader, 0, end_time, {kChannelName1});
  EXPECT_EQ(CheckCount(viewer_7), msg_num);
  ASSERT_FALSE(remove(kTestFile));
}

TEST(RecordTest, mult_iterator_test) {
  uint64_t msg_num = 200;
  uint64_t begin_time = 100000000;
  uint64_t step_time = 100000000;  // 100ms
  uint64_t end_time = begin_time + step_time * (msg_num - 1);
  ConstructRecord(msg_num, begin_time, step_time);

  auto reader = std::make_shared<RecordReader>(kTestFile);
  RecordViewer viewer(reader);
  EXPECT_TRUE(viewer.IsValid());
  EXPECT_EQ(begin_time, viewer.begin_time());
  EXPECT_EQ(end_time, viewer.end_time());

  auto bg = viewer.begin();  // #1 iterator

  uint64_t i = 0;
  for (auto& msg : viewer) {  // #2 iterator
    EXPECT_EQ(kChannelName1, msg.channel_name);
    EXPECT_EQ(begin_time + step_time * i, msg.time);
    EXPECT_EQ(std::to_string(i), msg.content);
    i++;
  }
  EXPECT_EQ(msg_num, i);
  ASSERT_FALSE(remove(kTestFile));
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
