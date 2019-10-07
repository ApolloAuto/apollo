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

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "cyber/cyber.h"
#include "cyber/message/py_message.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/py_wrapper/py_node.h"

using apollo::cyber::Time;
using apollo::cyber::message::PyMessageWrap;

apollo::cyber::PyReader *pr = nullptr;

int cbfun(const char *channel_name) {
  AINFO << "recv->[ " << channel_name << " ]";
  if (pr) {
    AINFO << "read->[ " << pr->read() << " ]";
  }
}

TEST(CyberNodeTest, create_reader) {
  EXPECT_TRUE(apollo::cyber::OK());
  apollo::cyber::proto::Chatter chat;
  apollo::cyber::PyNode node("listener");
  pr = node.create_reader("channel/chatter", chat.GetTypeName());
  EXPECT_EQ("apollo.cyber.proto.Chatter", chat.GetTypeName());
  EXPECT_NE(pr, nullptr);
  pr->register_func(cbfun);
  delete pr;
  pr = nullptr;
}

TEST(CyberNodeTest, create_writer) {
  EXPECT_TRUE(apollo::cyber::OK());
  auto msgChat = std::make_shared<apollo::cyber::proto::Chatter>();
  apollo::cyber::PyNode node("talker");
  apollo::cyber::PyWriter *pw =
      node.create_writer("channel/chatter", msgChat->GetTypeName(), 10);
  EXPECT_NE(pw, nullptr);

  EXPECT_TRUE(apollo::cyber::OK());
  uint64_t seq = 5;
  msgChat->set_timestamp(Time::Now().ToNanosecond());
  msgChat->set_lidar_timestamp(Time::Now().ToNanosecond());
  msgChat->set_seq(seq++);
  msgChat->set_content("Hello, apollo!");

  std::string org_data;
  msgChat->SerializeToString(&org_data);
  EXPECT_TRUE(pw->write(org_data));

  delete pw;
  pw = nullptr;
}

int main(int argc, char **argv) {
  apollo::cyber::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
