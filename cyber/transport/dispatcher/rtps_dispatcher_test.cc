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

#include "cyber/transport/dispatcher/rtps_dispatcher.h"

#include <memory>
#include "gtest/gtest.h"

#include "cyber/common/util.h"
#include "cyber/init.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/transport/common/identity.h"
#include "cyber/transport/qos/qos_profile_conf.h"
#include "cyber/transport/transport.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(RtpsDispatcherTest, add_listener) {
  auto dispatcher = RtpsDispatcher::Instance();
  RoleAttributes self_attr;
  self_attr.set_channel_name("add_listener");
  self_attr.set_channel_id(common::Hash("add_listener"));
  Identity self_id;
  self_attr.set_id(self_id.HashValue());
  self_attr.mutable_qos_profile()->CopyFrom(
      QosProfileConf::QOS_PROFILE_DEFAULT);

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

TEST(RtpsDispatcherTest, on_message) {
  auto dispatcher = RtpsDispatcher::Instance();
  RoleAttributes self_attr;
  self_attr.set_channel_name("channel_0");
  self_attr.set_channel_id(common::Hash("channel_0"));
  Identity self_id;
  self_attr.set_id(self_id.HashValue());
  self_attr.mutable_qos_profile()->CopyFrom(
      QosProfileConf::QOS_PROFILE_DEFAULT);

  auto recv_msg = std::make_shared<proto::Chatter>();
  dispatcher->AddListener<proto::Chatter>(
      self_attr, [&recv_msg](const std::shared_ptr<proto::Chatter>& msg,
                             const MessageInfo& msg_info) {
        (void)msg_info;
        recv_msg->CopyFrom(*msg);
      });

  auto transmitter = Transport::Instance()->CreateTransmitter<proto::Chatter>(
      self_attr, proto::OptionalMode::RTPS);
  EXPECT_NE(transmitter, nullptr);

  auto send_msg = std::make_shared<proto::Chatter>();
  send_msg->set_timestamp(123);
  send_msg->set_lidar_timestamp(456);
  send_msg->set_seq(789);
  send_msg->set_content("on_message");

  transmitter->Transmit(send_msg);
  sleep(1);

  EXPECT_EQ(recv_msg->timestamp(), send_msg->timestamp());
  EXPECT_EQ(recv_msg->lidar_timestamp(), send_msg->lidar_timestamp());
  EXPECT_EQ(recv_msg->seq(), send_msg->seq());
  EXPECT_EQ(recv_msg->content(), send_msg->content());
}

TEST(RtpsDispatcherTest, shutdown) {
  auto dispatcher = RtpsDispatcher::Instance();
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
