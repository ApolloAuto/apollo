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

#include "cyber/proto/record.pb.h"

#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

using apollo::cyber::message::RawMessage;
using ::apollo::cyber::record::RecordMessage;
using ::apollo::cyber::record::RecordReader;
using ::apollo::cyber::record::RecordWriter;

const char CHANNEL_NAME_1[] = "/test/channel1";
const char CHANNEL_NAME_2[] = "/test/channel2";
const char MESSAGE_TYPE_1[] = "apollo.cyber.proto.Test";
const char MESSAGE_TYPE_2[] = "apollo.cyber.proto.Channel";
const char PROTO_DESC[] = "1234567890";
const char STR_10B[] = "1234567890";
const char TEST_FILE[] = "test.record";

void test_write(const std::string &writefile) {
  RecordWriter writer;
  writer.SetSizeOfFileSegmentation(0);
  writer.SetIntervalOfFileSegmentation(0);
  writer.Open(writefile);
  writer.WriteChannel(CHANNEL_NAME_1, MESSAGE_TYPE_1, PROTO_DESC);
  for (uint32_t i = 0; i < 100; ++i) {
    auto msg = std::make_shared<RawMessage>("abc" + std::to_string(i));
    writer.WriteMessage(CHANNEL_NAME_1, msg, 888 + i);
  }
  writer.Close();
}

void test_read(const std::string &readfile) {
  RecordReader reader(readfile);
  RecordMessage message;
  uint64_t msg_count = reader.GetMessageNumber(CHANNEL_NAME_1);
  AINFO << "MSGTYPE: " << reader.GetMessageType(CHANNEL_NAME_1);
  AINFO << "MSGDESC: " << reader.GetProtoDesc(CHANNEL_NAME_1);

  // read all message
  uint64_t i = 0;
  uint64_t valid = 0;
  for (i = 0; i < msg_count; ++i) {
    if (reader.ReadMessage(&message)) {
      AINFO << "msg[" << i << "]-> "
            << "channel name: " << message.channel_name
            << "; content: " << message.content
            << "; msg time: " << message.time;
      valid++;
    } else {
      AERROR << "read msg[" << i << "] failed";
    }
  }
  AINFO << "static msg=================";
  AINFO << "MSG validmsg:totalcount: " << valid << ":" << msg_count;
}

int main(int argc, char *argv[]) {
  apollo::cyber::Init(argv[0]);
  test_write(TEST_FILE);
  sleep(1);
  test_read(TEST_FILE);
  return 0;
}
