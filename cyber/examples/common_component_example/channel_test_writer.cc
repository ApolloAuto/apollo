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

#include "cyber/cyber.h"
#include "cyber/examples/proto/examples.pb.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;
using apollo::cyber::examples::proto::Driver;

int main(int argc, char *argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create talker node
  auto talker_node = apollo::cyber::CreateNode("channel_test_writer");
  // create talker
  auto talker1 = talker_node->CreateWriter<Driver>("/apollo/test1");
  auto talker2 = talker_node->CreateWriter<Driver>("/apollo/test2");
  auto talker3 = talker_node->CreateWriter<Driver>("/apollo/test3");
  auto talker4 = talker_node->CreateWriter<Driver>("/apollo/test4");
  Rate rate(2.0);

  std::string content("apollo_test");
  while (apollo::cyber::OK()) {
    static uint64_t seq = 0;
    auto msg1 = std::make_shared<Driver>();
    msg1->set_timestamp(Time::Now().ToNanosecond());
    msg1->set_msg_id(seq);
    msg1->set_content(content + ": msg1: "  + std::to_string(seq));
    talker1->Write(msg1);

    auto msg2 = std::make_shared<Driver>();
    msg2->set_timestamp(Time::Now().ToNanosecond());
    msg2->set_msg_id(seq);
    msg2->set_content(content + ": msg2: " + std::to_string(seq));
    talker2->Write(msg2);

    auto msg3 = std::make_shared<Driver>();
    msg3->set_timestamp(Time::Now().ToNanosecond());
    msg3->set_msg_id(seq);
    msg3->set_content(content + ": msg3: " + std::to_string(seq));
    talker3->Write(msg3);

    auto msg4 = std::make_shared<Driver>();
    msg4->set_timestamp(Time::Now().ToNanosecond());
    msg4->set_msg_id(seq);
    msg4->set_content(content + ": msg4: " + std::to_string(seq));
    talker4->Write(msg4);

    AINFO << "/apollo/test sent message, seq=" << seq << ";";
    rate.Sleep();
    seq++;
  }
  return 0;
}
