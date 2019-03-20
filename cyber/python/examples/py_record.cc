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

#include "cyber/cyber.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/py_wrapper/py_record.h"

const char TEST_RECORD_FILE[] = "test02.record";
const char CHAN_1[] = "channel/chatter";
const char CHAN_2[] = "/test2";
const char MSG_TYPE[] = "apollo.cyber.proto.Test";
const char STR_10B[] = "1234567890";
const char TEST_FILE[] = "test.record";

void test_write(const std::string &writefile) {
  apollo::cyber::record::PyRecordWriter rec_writer;
  AINFO << "++++ begin writer";
  rec_writer.Open(writefile);
  rec_writer.SetSizeOfFileSegmentation(0);
  rec_writer.SetIntervalOfFileSegmentation(0);
  rec_writer.WriteChannel(CHAN_1, MSG_TYPE, STR_10B);
  rec_writer.WriteMessage(CHAN_1, STR_10B, 1000);
  rec_writer.Close();
}

void test_read(const std::string &readfile) {
  apollo::cyber::record::PyRecordReader rec_reader(readfile);
  AINFO << "++++ begin reading";
  sleep(1);
  int count = 0;

  apollo::cyber::record::BagMessage bag_msg = rec_reader.ReadMessage();
  while (!bag_msg.end) {
    AINFO << "========================";
    std::string channel_name = bag_msg.channel_name;
    AINFO << "read msg[" << count << "]";
    AINFO << "cur channel:[" << channel_name
          << "] msg total:" << rec_reader.GetMessageNumber(channel_name) << "] "
          << "cur msg:[ " << bag_msg.data << " ]";
    AINFO << "curMsgTime: " << bag_msg.timestamp;
    AINFO << "msg type:" << bag_msg.data_type;
    AINFO << "msg protoDesc:" << rec_reader.GetProtoDesc(channel_name);
    count++;
    bag_msg = rec_reader.ReadMessage();
  }

  AINFO << "reader msg count = " << count;
}

// ./py_record readfile1. only read readfile1
// other write & read
int main(int argc, char *argv[]) {
  apollo::cyber::Init("cyber_python");
  if (argc == 2) {
    std::string readfile(argv[1]);
    AINFO << "beging to read: " << readfile;
    test_read(readfile);
    return 1;
  }
  test_write(TEST_RECORD_FILE);
  sleep(1);
  test_read(TEST_RECORD_FILE);

  return 0;
}
