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

#include <string>

#include "cybertron/cybertron.h"
#include "cybertron/proto/chatter.pb.h"
#include "gtest/gtest.h"
#include "python/wrapper/py_record.h"

const char TEST_RECORD_FILE[] = "test02.record";
const char CHAN_1[] = "channel/chatter";
const char CHAN_2[] = "/test2";
const char MSG_TYPE[] = "apollo.cybertron.proto.Test";
const char STR_10B[] = "1234567890";
const char TEST_FILE[] = "test.record";

TEST(CyberRecordTest, record_readerwriter) {
  apollo::cybertron::record::PyRecordWriter rec_writer;

  EXPECT_TRUE(rec_writer.Open(TEST_RECORD_FILE));
  rec_writer.WriteChannel(CHAN_1, MSG_TYPE, STR_10B);
  rec_writer.WriteMessage(CHAN_1, STR_10B, 888);
  rec_writer.Close();

  // read
  apollo::cybertron::record::PyRecordReader rec_reader;
  AINFO << "++++ begin reading";
  EXPECT_TRUE(rec_reader.Open(TEST_RECORD_FILE));

  sleep(1);
  int count = 0;
  bool bReadMsg = true;
  EXPECT_FALSE(rec_reader.EndOfFile());
  EXPECT_TRUE(rec_reader.ReadMessage());

  std::string channel_name = rec_reader.CurrentMessageChannelName();
  EXPECT_EQ(CHAN_1, channel_name);
  EXPECT_EQ(STR_10B, rec_reader.CurrentRawMessage());
  EXPECT_EQ(1, rec_reader.GetMessageNumber(channel_name));
  EXPECT_EQ(888, rec_reader.CurrentMessageTime());
  EXPECT_EQ(MSG_TYPE, rec_reader.GetMessageType(channel_name));
  std::string header_str = rec_reader.GetHeaderString();
  apollo::cybertron::proto::Header header;
  header.ParseFromString(header_str);
  EXPECT_EQ(1, header.major_version());
  EXPECT_EQ(0, header.minor_version());
  EXPECT_EQ(1, header.chunk_number());
  EXPECT_EQ(1, header.channel_number());
  EXPECT_TRUE(header.is_complete());

  EXPECT_TRUE(rec_reader.EndOfFile());
  rec_reader.Close();
}

int main(int argc, char** argv) {
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
