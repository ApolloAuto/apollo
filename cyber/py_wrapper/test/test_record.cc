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

#include <set>
#include <string>

#include "cyber/cyber.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/py_wrapper/py_record.h"
#include "gtest/gtest.h"

const char TEST_RECORD_FILE[] = "test02.record";
const char CHAN_1[] = "channel/chatter";
const char CHAN_2[] = "/test2";
const char MSG_TYPE[] = "apollo.cyber.proto.Test";
const char STR_10B[] = "1234567890";
const char TEST_FILE[] = "test.record";

TEST(CyberRecordTest, record_readerwriter) {
  apollo::cyber::record::PyRecordWriter rec_writer;

  EXPECT_TRUE(rec_writer.Open(TEST_RECORD_FILE));
  rec_writer.WriteChannel(CHAN_1, MSG_TYPE, STR_10B);
  rec_writer.WriteMessage(CHAN_1, STR_10B, 888);
  EXPECT_EQ(1, rec_writer.GetMessageNumber(CHAN_1));
  EXPECT_EQ(MSG_TYPE, rec_writer.GetMessageType(CHAN_1));
  EXPECT_EQ(STR_10B, rec_writer.GetProtoDesc(CHAN_1));
  rec_writer.Close();

  // read
  apollo::cyber::record::PyRecordReader rec_reader(TEST_RECORD_FILE);
  AINFO << "++++ begin reading";

  sleep(1);
  int count = 0;
  apollo::cyber::record::BagMessage bag_msg = rec_reader.ReadMessage();

  std::set<std::string> channel_list = rec_reader.GetChannelList();
  EXPECT_EQ(1, channel_list.size());
  EXPECT_EQ(CHAN_1, *channel_list.begin());

  std::string channel_name = bag_msg.channel_name;
  EXPECT_EQ(CHAN_1, channel_name);
  EXPECT_EQ(STR_10B, bag_msg.data);
  EXPECT_EQ(1, rec_reader.GetMessageNumber(channel_name));
  EXPECT_EQ(888, bag_msg.timestamp);
  EXPECT_EQ(MSG_TYPE, bag_msg.data_type);
  EXPECT_EQ(MSG_TYPE, rec_reader.GetMessageType(channel_name));
  std::string header_str = rec_reader.GetHeaderString();
  apollo::cyber::proto::Header header;
  header.ParseFromString(header_str);
  EXPECT_EQ(1, header.major_version());
  EXPECT_EQ(0, header.minor_version());
  EXPECT_EQ(1, header.chunk_number());
  EXPECT_EQ(1, header.channel_number());
  EXPECT_TRUE(header.is_complete());
}

int main(int argc, char** argv) {
  apollo::cyber::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
