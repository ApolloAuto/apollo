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

#include "cyber/transport/dispatcher/shm_dispatcher.h"

#include <memory>
#include "gtest/gtest.h"

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/util.h"
#include "cyber/init.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/transport/common/identity.h"
#include "cyber/transport/transport.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(ShmDispatcherTest, add_listener) {
  auto dispatcher = ShmDispatcher::Instance();
  RoleAttributes self_attr;
  self_attr.set_channel_name("add_listener");
  self_attr.set_channel_id(common::Hash("add_listener"));
  Identity self_id;
  self_attr.set_id(self_id.HashValue());

  dispatcher->AddListener<proto::Chatter>(
      self_attr,
      [](const std::shared_ptr<proto::Chatter>&, const MessageInfo&) {});

  RoleAttributes oppo_attr;
  oppo_attr.CopyFrom(self_attr);
  Identity oppo_id;
  oppo_attr.set_id(oppo_id.HashValue());

  dispatcher->AddListener<proto::Chatter>(
      self_attr, oppo_attr,
      [](const std::shared_ptr<proto::Chatter>&, const MessageInfo&) {});
}

TEST(ShmDispatcherTest, on_message) {
  auto dispatcher = ShmDispatcher::Instance();

  RoleAttributes oppo_attr;
  oppo_attr.set_host_name(common::GlobalData::Instance()->HostName());
  oppo_attr.set_host_ip(common::GlobalData::Instance()->HostIp());
  oppo_attr.set_channel_name("on_message");
  oppo_attr.set_channel_id(common::Hash("on_message"));
  Identity oppo_id;
  oppo_attr.set_id(oppo_id.HashValue());

  auto transmitter =
      Transport::Instance()->CreateTransmitter<message::RawMessage>(
          oppo_attr, proto::OptionalMode::SHM);
  EXPECT_NE(transmitter, nullptr);

  auto send_msg = std::make_shared<message::RawMessage>("raw_message");
  transmitter->Transmit(send_msg);

  sleep(1);

  RoleAttributes self_attr;
  self_attr.set_channel_name("on_message");
  self_attr.set_channel_id(common::Hash("on_message"));
  Identity self_id;
  self_attr.set_id(self_id.HashValue());

  auto recv_msg = std::make_shared<message::RawMessage>();
  dispatcher->AddListener<message::RawMessage>(
      self_attr, [&recv_msg](const std::shared_ptr<message::RawMessage>& msg,
                             const MessageInfo& msg_info) {
        (void)msg_info;
        recv_msg->message = msg->message;
      });

  transmitter->Transmit(send_msg);

  sleep(1);
  EXPECT_EQ(recv_msg->message, send_msg->message);
}

TEST(ShmDispatcherTest, shutdown) {
  auto dispatcher = ShmDispatcher::Instance();
  dispatcher->Shutdown();

  // repeated call
  dispatcher->Shutdown();
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  apollo::cyber::transport::Transport::Instance();
  auto res = RUN_ALL_TESTS();
  apollo::cyber::transport::Transport::Instance()->Shutdown();
  return res;
}
