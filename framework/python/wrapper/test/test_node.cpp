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

#include "cybertron/cybertron.h"
#include "cybertron/message/py_message.h"
#include "cybertron/proto/chatter.pb.h"
#include "gtest/gtest.h"
#include "python/wrapper/py_node.h"

using apollo::cybertron::message::PyMessageWrap;
using apollo::cybertron::Time;

apollo::cybertron::PyReader *pr = NULL;

int cbfun(const char *channel_name) {
  AINFO << "recv->[ " << channel_name << " ]";
  if (pr) AINFO << "read->[ " << pr->read() << " ]";
}

TEST(CyberNodeTest, create_reader) {
  EXPECT_TRUE(apollo::cybertron::OK());
  apollo::cybertron::proto::Chatter chat;
  apollo::cybertron::PyNode node("listener");
  pr = node.create_reader("channel/chatter", chat.GetTypeName());
  EXPECT_EQ("apollo.cybertron.proto.Chatter", chat.GetTypeName());
  EXPECT_TRUE(pr != nullptr);
  pr->register_func(cbfun);
  delete pr;
  pr = nullptr;
}

TEST(CyberNodeTest, create_writer) {
  EXPECT_TRUE(apollo::cybertron::OK());
  auto msgChat = std::make_shared<apollo::cybertron::proto::Chatter>();
  apollo::cybertron::PyNode node("talker");
  apollo::cybertron::PyWriter *pw =
      node.create_writer("channel/chatter", msgChat->GetTypeName(), 10);
  EXPECT_TRUE(pw != nullptr);

  EXPECT_TRUE(apollo::cybertron::OK());
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
  apollo::cybertron::Init(argv[0]);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
