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

#include "cyber/py_wrapper/py_cyber.h"

#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "cyber/cyber.h"
#include "cyber/message/py_message.h"
#include "cyber/proto/unit_test.pb.h"

namespace apollo {
namespace cyber {

TEST(PyCyberTest, init_ok) {
  EXPECT_TRUE(py_ok());
  EXPECT_TRUE(OK());
}

TEST(PyCyberTest, create_reader) {
  EXPECT_TRUE(OK());
  proto::Chatter chat;
  PyNode node("listener");
  std::unique_ptr<PyReader> pr(
      node.create_reader("channel/chatter", chat.GetTypeName()));
  EXPECT_EQ("apollo.cyber.proto.Chatter", chat.GetTypeName());
  EXPECT_NE(pr, nullptr);
  pr->register_func([](const char* channel_name) -> int {
    AINFO << "recv->[ " << channel_name << " ]";
    return 0;
  });
}

TEST(PyCyberTest, create_writer) {
  EXPECT_TRUE(OK());
  auto msgChat = std::make_shared<proto::Chatter>();
  PyNode node("talker");
  std::unique_ptr<PyWriter> pw(
      node.create_writer("channel/chatter", msgChat->GetTypeName(), 10));
  EXPECT_NE(pw, nullptr);

  EXPECT_TRUE(OK());
  uint64_t seq = 5;
  msgChat->set_timestamp(Time::Now().ToNanosecond());
  msgChat->set_lidar_timestamp(Time::Now().ToNanosecond());
  msgChat->set_seq(seq++);
  msgChat->set_content("Hello, apollo!");

  std::string org_data;
  msgChat->SerializeToString(&org_data);
  EXPECT_TRUE(pw->write(org_data));
}

}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::py_init("py_init_test");
  const int ret_val = RUN_ALL_TESTS();
  apollo::cyber::py_shutdown();

  return ret_val;
}
