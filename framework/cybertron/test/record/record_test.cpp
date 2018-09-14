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

#include "cybertron/record/record_reader.h"
#include "cybertron/record/record_writer.h"
#include "cybertron/record/header_builder.h"

#include <gtest/gtest.h>
#include <unistd.h>
#include <atomic>

#include "cybertron/common/log.h"

namespace apollo {
namespace cybertron {
namespace record {

const std::string CHANNEL_NAME = "/test/channel1";
const std::string MESSAGE_TYPE = "apollo.cybertron.proto.Test";
const std::string PROTO_DESC = "1234567890";
const std::string STR_10B = "1234567890";
const std::string TEST_FILE = "test.record";
const uint64_t TIME = 1e9;

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

  ASSERT_TRUE(rw->WriteChannel(CHANNEL_NAME, MESSAGE_TYPE, PROTO_DESC));
  ASSERT_EQ(0, rw->GetMessageNumber(CHANNEL_NAME));
  ASSERT_EQ(MESSAGE_TYPE, rw->GetMessageType(CHANNEL_NAME));
  ASSERT_EQ(PROTO_DESC, rw->GetProtoDesc(CHANNEL_NAME));

  std::shared_ptr<RawMessage> rm(new RawMessage(STR_10B));
  ASSERT_TRUE(rw->WriteMessage(CHANNEL_NAME, rm, TIME));
  ASSERT_EQ(1, rw->GetMessageNumber(CHANNEL_NAME));

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

  ASSERT_EQ(CHANNEL_NAME, rr->CurrentMessageChannelName());
  ASSERT_EQ(STR_10B, rr->CurrentRawMessage()->message);
  ASSERT_EQ(TIME, rr->CurrentMessageTime());

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
