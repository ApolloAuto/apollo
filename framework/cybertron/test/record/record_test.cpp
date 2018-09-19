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

#include "cybertron/record/header_builder.h"
#include "cybertron/record/record_reader.h"
#include "cybertron/record/record_writer.h"

#include <gtest/gtest.h>
#include <unistd.h>
#include <atomic>
#include <memory>
#include <string>

#include "cybertron/common/log.h"

namespace apollo {
namespace cybertron {
namespace record {

const char CHANNEL_NAME_1[] = "/test/channel1";
const char CHANNEL_NAME_2[] = "/test/channel2";
const char MESSAGE_TYPE_1[] = "apollo.cybertron.proto.Test";
const char MESSAGE_TYPE_2[] = "apollo.cybertron.proto.Channel";
const char PROTO_DESC[] = "1234567890";
const char STR_10B[] = "1234567890";
const char TEST_FILE[] = "test.record";
const uint64_t TIME_1 = 1000 * 1e6;
const uint64_t TIME_2 = 1010 * 1e6;
const uint64_t TIME_3 = 1020 * 1e6;

TEST(RecordTest, TestOneMessageFile) {
  // writer
  RecordWriter* rw = new RecordWriter();
  ASSERT_FALSE(rw->is_opened_);
  ASSERT_EQ("", rw->file_);
  ASSERT_EQ("", rw->path_);
  ASSERT_EQ(nullptr, rw->file_writer_);

  ASSERT_TRUE(rw->Open(TEST_FILE));
  ASSERT_TRUE(rw->is_opened_);
  ASSERT_EQ(TEST_FILE, rw->file_);
  ASSERT_EQ(TEST_FILE, rw->path_);
  ASSERT_TRUE(rw->file_writer_->ofstream_.is_open());

  ASSERT_TRUE(rw->WriteChannel(CHANNEL_NAME_1, MESSAGE_TYPE_1, PROTO_DESC));
  ASSERT_EQ(0, rw->GetMessageNumber(CHANNEL_NAME_1));
  ASSERT_EQ(MESSAGE_TYPE_1, rw->GetMessageType(CHANNEL_NAME_1));
  ASSERT_EQ(PROTO_DESC, rw->GetProtoDesc(CHANNEL_NAME_1));

  std::shared_ptr<RawMessage> rm(new RawMessage(STR_10B));
  ASSERT_TRUE(rw->WriteMessage(CHANNEL_NAME_1, rm, TIME_1));
  ASSERT_EQ(1, rw->GetMessageNumber(CHANNEL_NAME_1));

  delete rw;

  // reader
  RecordReader* rr = new RecordReader();
  ASSERT_FALSE(rr->is_opened_);
  ASSERT_EQ("", rr->file_);
  ASSERT_EQ("", rr->path_);
  ASSERT_EQ(nullptr, rr->file_reader_);

  ASSERT_TRUE(rr->Open(TEST_FILE));
  ASSERT_TRUE(rr->is_opened_);
  ASSERT_EQ(TEST_FILE, rr->file_);
  ASSERT_EQ(TEST_FILE, rr->path_);
  ASSERT_TRUE(rr->file_reader_->ifstream_.is_open());

  sleep(1);

  ASSERT_EQ("", rr->CurrentMessageChannelName());
  ASSERT_EQ("", rr->CurrentRawMessage()->message);
  ASSERT_EQ(0, rr->CurrentMessageTime());

  ASSERT_FALSE(rr->EndOfFile());
  ASSERT_TRUE(rr->ReadMessage());

  ASSERT_EQ(CHANNEL_NAME_1, rr->CurrentMessageChannelName());
  ASSERT_EQ(STR_10B, rr->CurrentRawMessage()->message);
  ASSERT_EQ(TIME_1, rr->CurrentMessageTime());

  ASSERT_TRUE(rr->EndOfFile());
  ASSERT_FALSE(rr->ReadMessage());

  delete rr;
}

TEST(RecordTest, TestMutiMessageFile) {
  // writer
  RecordWriter* rw = new RecordWriter();
  ASSERT_FALSE(rw->is_opened_);
  ASSERT_EQ("", rw->file_);
  ASSERT_EQ("", rw->path_);
  ASSERT_EQ(nullptr, rw->file_writer_);

  ASSERT_TRUE(rw->Open(TEST_FILE));
  ASSERT_TRUE(rw->is_opened_);
  ASSERT_EQ(TEST_FILE, rw->file_);
  ASSERT_EQ(TEST_FILE, rw->path_);
  ASSERT_TRUE(rw->file_writer_->ofstream_.is_open());

  ASSERT_TRUE(rw->WriteChannel(CHANNEL_NAME_1, MESSAGE_TYPE_1, PROTO_DESC));
  ASSERT_EQ(0, rw->GetMessageNumber(CHANNEL_NAME_1));
  ASSERT_EQ(MESSAGE_TYPE_1, rw->GetMessageType(CHANNEL_NAME_1));
  ASSERT_EQ(PROTO_DESC, rw->GetProtoDesc(CHANNEL_NAME_1));

  ASSERT_TRUE(rw->WriteChannel(CHANNEL_NAME_2, MESSAGE_TYPE_2, PROTO_DESC));
  ASSERT_EQ(0, rw->GetMessageNumber(CHANNEL_NAME_2));
  ASSERT_EQ(MESSAGE_TYPE_2, rw->GetMessageType(CHANNEL_NAME_2));
  ASSERT_EQ(PROTO_DESC, rw->GetProtoDesc(CHANNEL_NAME_2));

  std::shared_ptr<RawMessage> rm(new RawMessage(STR_10B));

  ASSERT_TRUE(rw->WriteMessage(CHANNEL_NAME_1, rm, TIME_1));
  ASSERT_EQ(1, rw->GetMessageNumber(CHANNEL_NAME_1));

  Channel pbmsg;
  pbmsg.set_name(CHANNEL_NAME_2);
  pbmsg.set_message_type(MESSAGE_TYPE_2);
  pbmsg.set_proto_desc(PROTO_DESC);
  std::string pbmsg_content("");
  pbmsg.SerializeToString(&pbmsg_content);

  ASSERT_TRUE(rw->WritePbMessage<Channel>(CHANNEL_NAME_2, pbmsg, TIME_2, PROTO_DESC));
  ASSERT_EQ(1, rw->GetMessageNumber(CHANNEL_NAME_2));

  ASSERT_TRUE(rw->WriteMessage(CHANNEL_NAME_1, rm, TIME_3));
  ASSERT_EQ(2, rw->GetMessageNumber(CHANNEL_NAME_1));

  delete rw;

  // reader
  RecordReader* rr = new RecordReader();
  ASSERT_FALSE(rr->is_opened_);
  ASSERT_EQ("", rr->file_);
  ASSERT_EQ("", rr->path_);
  ASSERT_EQ(nullptr, rr->file_reader_);

  std::vector<std::string> channel_vec = std::vector<std::string>();
  channel_vec.push_back(CHANNEL_NAME_2);
  ASSERT_TRUE(rr->Open(TEST_FILE, TIME_1, TIME_2, channel_vec));
  ASSERT_TRUE(rr->is_opened_);
  ASSERT_EQ(TEST_FILE, rr->file_);
  ASSERT_EQ(TEST_FILE, rr->path_);
  ASSERT_TRUE(rr->file_reader_->ifstream_.is_open());

  sleep(1);

  ASSERT_EQ("", rr->CurrentMessageChannelName());
  ASSERT_EQ("", rr->CurrentRawMessage()->message);
  ASSERT_EQ(0, rr->CurrentMessageTime());

  ASSERT_FALSE(rr->EndOfFile());
  ASSERT_TRUE(rr->ReadMessage());

  ASSERT_EQ(CHANNEL_NAME_2, rr->CurrentMessageChannelName());
  ASSERT_EQ(pbmsg_content, rr->CurrentRawMessage()->message);
  ASSERT_EQ(TIME_2, rr->CurrentMessageTime());

  ASSERT_TRUE(rr->EndOfFile());
  ASSERT_FALSE(rr->ReadMessage());

  delete rr;
}

}  // namespace record
}  // namespace cybertron
}  // namespace apollo

int main(int argc, char** argv) {
  testing::GTEST_FLAG(catch_exceptions) = 1;
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  return RUN_ALL_TESTS();
}
