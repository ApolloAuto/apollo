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

#include "cybertron/cybertron.h"
#include "cybertron/proto/chatter.pb.h"
#include "python/wrapper/py_record.h"


const std::string TEST_RECORD_FILE = "test02.record";

const std::string CHAN_1 = "channel/chatter";
const std::string CHAN_2 = "/test2";
const std::string MSG_TYPE = "apollo.cybertron.proto.Test";
const std::string STR_10B = "1234567890";
const std::string TEST_FILE = "test.record";

void test_write(const std::string &writefile) {
  apollo::cybertron::record::PyRecordWriter rec_writer;
  AINFO << "++++ begin writer";
  rec_writer.Open(writefile);

  rec_writer.WriteChannel(CHAN_1, MSG_TYPE, STR_10B);

  SingleMessage msg1;

  msg1.set_channel_name(CHAN_1);
  msg1.set_content(STR_10B);
  msg1.set_time(888);
  std::string msg_str = msg1.SerializeAsString();
  rec_writer.WriteMessage(msg_str);
  rec_writer.WriteMessage(msg_str);
  rec_writer.Close();
}

void test_read(const std::string &readfile) {
  apollo::cybertron::record::PyRecordReader rec_reader;
  AINFO << "++++ begin reading";
  if (!rec_reader.Open(readfile)) {
    AERROR << "reader open failed!";
    return ;
  }
  sleep(1);
  int count = 0;
  bool bReadMsg = true;
  while (!rec_reader.EndOfFile()) {
    bReadMsg = rec_reader.ReadMessage();
    if (bReadMsg) {
      AINFO << "========================";
      std::string channel_name = rec_reader.CurrentMessageChannelName();
      AINFO << "read msg[" << count << "]";
      AINFO << "cur channel:[" <<  channel_name
      << "] msg total:" << rec_reader.GetMessageNumber(channel_name)
      << "] " << "cur msg:[ " << rec_reader.CurrentRawMessage() << " ]";
      AINFO << "curMsgTime: " << rec_reader.CurrentMessageTime();
      AINFO << "msg type:" << rec_reader.GetMessageType(channel_name);
      AINFO << "msg protoDesc:" << rec_reader.GetProtoDesc(channel_name);
      count++;
    }
  }
  rec_reader.Close();
  AINFO << "reader msg count = " << count;
}
int main(int argc, char *argv[]) {
  apollo::cybertron::Init("cyber_python");
  test_write(TEST_RECORD_FILE);
  sleep(1);
  test_read(TEST_RECORD_FILE);


  return 0;
}
