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
#include "cybertron/record/header_builder.h"
#include "cybertron/record/record_file.h"

#include <gtest/gtest.h>
#include <unistd.h>
#include <atomic>

#include "cybertron/common/log.h"

namespace apollo {
namespace cybertron {
namespace record {

const std::string CHAN_1 = "/test1";
const std::string CHAN_2 = "/test2";
const std::string MSG_TYPE = "apollo.cybertron.proto.Test";
const std::string STR_10B = "1234567890";
const std::string TEST_FILE = "test.record";

TEST(RecordReaderTest, TestOneMessageFile) {
  // writer open one message file
  RecordFileWriter* rfw = new RecordFileWriter();
  ASSERT_TRUE(rfw->Open(TEST_FILE));
  ASSERT_EQ(TEST_FILE, rfw->path_);
  ASSERT_NE(nullptr, rfw->ofstream_);
  ASSERT_TRUE(rfw->ofstream_.is_open());
  ASSERT_NE(nullptr, rfw->chunk_active_);
  ASSERT_NE(nullptr, rfw->chunk_flush_);
  ASSERT_TRUE(rfw->is_writing_);
  ASSERT_NE(nullptr, rfw->flush_thread_);

  // write header section
  HeaderBuilder* hb = new HeaderBuilder();
  hb->BuildSegmentPart(0, 0);
  hb->BuildChunkPart(0, 0);
  Header hdr1 = hb->GetHeader();
  ASSERT_TRUE(rfw->WriteHeader(hdr1));
  ASSERT_FALSE(rfw->header_.is_complete());

  // write channel section
  Channel chan1;
  chan1.set_name(CHAN_1);
  chan1.set_message_type(MSG_TYPE);
  chan1.set_proto_desc(STR_10B);
  ASSERT_TRUE(rfw->WriteChannel(chan1));

  // write chunk section
  SingleMessage msg1;
  msg1.set_channel_name(chan1.name());
  msg1.set_content(STR_10B);
  msg1.set_time(1e9);
  ASSERT_TRUE(rfw->AddSingleMessage(msg1));
  ASSERT_EQ(1, rfw->channel_message_number_map_[chan1.name()]);
  ChunkHeader ckh1 = rfw->chunk_active_->header_;
  ChunkBody ckb1 = rfw->chunk_active_->body_;

  // writer close one message file
  rfw->Close();
  ASSERT_TRUE(rfw->header_.is_complete());
  ASSERT_EQ(1, rfw->header_.chunk_number());
  ASSERT_EQ(1e9, rfw->header_.begin_time());
  ASSERT_EQ(1e9, rfw->header_.end_time());
  ASSERT_EQ(1, rfw->header_.message_number());
  hdr1 = rfw->header_;
  delete rfw;

  // header open one message file
  RecordReader* rr = new RecordReader();
  ASSERT_TRUE(rr->Open(TEST_FILE));
  ASSERT_EQ(TEST_FILE, rr->file_);
  ASSERT_EQ(TEST_FILE, rr->path_);

  sleep(1);

  ASSERT_EQ("", rr->CurrentMessageChannelName());
  ASSERT_EQ("", rr->CurrentRawMessage()->message);
  ASSERT_EQ(0, rr->CurrentMessageTime());

  ASSERT_FALSE(rr->EndOfFile());
  ASSERT_TRUE(rr->ReadMessage());

  ASSERT_EQ(chan1.name(), rr->CurrentMessageChannelName());
  ASSERT_EQ(STR_10B, rr->CurrentRawMessage()->message);
  ASSERT_EQ(1e9, rr->CurrentMessageTime());

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
