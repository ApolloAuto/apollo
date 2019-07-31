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

#include "cyber/transport/dispatcher/intra_dispatcher.h"

#include <gtest/gtest.h>
#include <memory>

#include "cyber/common/util.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/transport/common/identity.h"

namespace apollo {
namespace cyber {
namespace transport {

constexpr char kContent[] = "on_message";

TEST(IntraDispatcherTest, on_chatter_message) {
  constexpr char kChannelName[] = "on_chatter_message_channel";
  static const std::size_t kChannelNameHash = common::Hash(kChannelName);

  auto dispatcher = IntraDispatcher::Instance();

  auto send_pb_msg = std::make_shared<proto::Chatter>();
  send_pb_msg->set_timestamp(123);
  send_pb_msg->set_lidar_timestamp(456);
  send_pb_msg->set_seq(789);
  send_pb_msg->set_content(kContent);

  RoleAttributes attr;
  attr.set_channel_name(kChannelName);
  attr.set_channel_id(kChannelNameHash);
  Identity pb_id;
  attr.set_id(pb_id.HashValue());
  auto recv_pb_msg = std::make_shared<proto::Chatter>();
  dispatcher->AddListener<proto::Chatter>(
      attr, [&recv_pb_msg](const std::shared_ptr<proto::Chatter>& msg,
                           const MessageInfo& msg_info) {
        (void)msg_info;
        recv_pb_msg->CopyFrom(*msg);
      });
  MessageInfo msg_info;
  dispatcher->OnMessage(kChannelNameHash, send_pb_msg, msg_info);

  EXPECT_EQ(recv_pb_msg->timestamp(), 123);
  EXPECT_EQ(recv_pb_msg->lidar_timestamp(), 456);
  EXPECT_EQ(recv_pb_msg->seq(), 789);
  EXPECT_EQ(recv_pb_msg->content(), kContent);
}

TEST(IntraDispatcherTest, on_raw_message) {
  constexpr char kChannelName[] = "on_raw_message_channel";
  static const std::size_t kChannelNameHash = common::Hash(kChannelName);

  auto dispatcher = IntraDispatcher::Instance();

  RoleAttributes attr;
  attr.set_channel_name(kChannelName);
  attr.set_channel_id(kChannelNameHash);
  Identity raw_id;
  attr.set_id(raw_id.HashValue());
  auto recv_raw_msg = std::make_shared<message::RawMessage>();
  dispatcher->AddListener<message::RawMessage>(
      attr, [&recv_raw_msg](const std::shared_ptr<message::RawMessage>& msg,
                            const MessageInfo& msg_info) {
        (void)msg_info;
        recv_raw_msg->message = msg->message;
      });

  auto send_raw_msg = std::make_shared<message::RawMessage>(kContent);
  MessageInfo msg_info;
  dispatcher->OnMessage(kChannelNameHash, send_raw_msg, msg_info);

  EXPECT_EQ(recv_raw_msg->message, send_raw_msg->message);
}

TEST(IntraDispatcherTest, raw_msg_pb_dispatcher_message) {
  constexpr char kChannelName[] = "raw_msg_pb_dispatcher_message_channel";
  static const std::size_t kChannelNameHash = common::Hash(kChannelName);

  auto dispatcher = IntraDispatcher::Instance();

  RoleAttributes attr;
  attr.set_channel_name(kChannelName);
  attr.set_channel_id(kChannelNameHash);
  Identity raw_id;
  attr.set_id(raw_id.HashValue());
  auto recv_pb_msg = std::make_shared<proto::Chatter>();
  dispatcher->AddListener<proto::Chatter>(
      attr, [&recv_pb_msg](const std::shared_ptr<proto::Chatter>& msg,
                           const MessageInfo& msg_info) {
        (void)msg_info;
        recv_pb_msg = msg;
      });

  proto::Chatter send_pb_msg;
  send_pb_msg.set_content(kContent);
  const std::string send_pb_msg_str = send_pb_msg.SerializeAsString();
  auto send_raw_msg = std::make_shared<message::RawMessage>(send_pb_msg_str);
  MessageInfo msg_info;
  dispatcher->OnMessage(kChannelNameHash, send_raw_msg, msg_info);

  std::string recv_pb_msg_str;
  recv_pb_msg->SerializeToString(&recv_pb_msg_str);
  EXPECT_EQ(recv_pb_msg_str, send_raw_msg->message);
}

TEST(IntraDispatcherTest, pb_msg_raw_dispatcher) {
  constexpr char kChannelName[] = "pb_msg_raw_dispatcher_channel";
  static const std::size_t kChannelNameHash = common::Hash(kChannelName);

  auto dispatcher = IntraDispatcher::Instance();

  RoleAttributes attr;
  attr.set_channel_name(kChannelName);
  attr.set_channel_id(kChannelNameHash);
  Identity raw_id;
  attr.set_id(raw_id.HashValue());
  auto recv_raw_msg = std::make_shared<message::RawMessage>();
  dispatcher->AddListener<message::RawMessage>(
      attr, [&recv_raw_msg](const std::shared_ptr<message::RawMessage>& msg,
                            const MessageInfo& msg_info) {
        (void)msg_info;
        recv_raw_msg->message = msg->message;
      });

  auto send_pb_msg = std::make_shared<proto::Chatter>();
  send_pb_msg->set_content(kContent);
  MessageInfo msg_info;
  dispatcher->OnMessage(kChannelNameHash, send_pb_msg, msg_info);

  std::string send_pb_msg_str;
  send_pb_msg->SerializeToString(&send_pb_msg_str);
  EXPECT_EQ(send_pb_msg_str, recv_raw_msg->message);
}

TEST(IntraDispatcherTest, chatter_msg_unit_test_dispatcher) {
  constexpr char kChannelName[] = "chatter_msg_unit_test_dispatcher_channel";
  static const std::size_t kChannelNameHash = common::Hash(kChannelName);

  auto dispatcher = IntraDispatcher::Instance();

  RoleAttributes attr;
  attr.set_channel_name(kChannelName);
  attr.set_channel_id(kChannelNameHash);
  Identity raw_id;
  attr.set_id(raw_id.HashValue());
  dispatcher->AddListener<proto::UnitTest>(
      attr, [](const std::shared_ptr<proto::UnitTest>& msg,
               const MessageInfo& msg_info) {
        (void)msg_info;
        (void)msg;
      });

  auto send_pb_msg = std::make_shared<proto::Chatter>();
  send_pb_msg->set_content(kContent);
  MessageInfo msg_info;
  ASSERT_DEATH(
      dispatcher->OnMessage(kChannelNameHash, send_pb_msg, msg_info),
      "please ensure that readers with the same channel\\[\\] in the same "
      "process have the same message type");
}

TEST(IntraDispatcherTest, raw_msg_int_dispatcher) {
  constexpr char kChannelName[] = "raw_msg_int_dispatcher_channel";
  static const std::size_t kChannelNameHash = common::Hash(kChannelName);

  auto dispatcher = IntraDispatcher::Instance();

  RoleAttributes attr;
  attr.set_channel_name(kChannelName);
  attr.set_channel_id(kChannelNameHash);
  Identity raw_id;
  attr.set_id(raw_id.HashValue());
  dispatcher->AddListener<int>(
      attr, [](const std::shared_ptr<int>& msg, const MessageInfo& msg_info) {
        (void)msg_info;
        (void)msg;
      });

  auto send_pb_msg = std::make_shared<message::RawMessage>(kContent);
  MessageInfo msg_info;
  ASSERT_DEATH(dispatcher->OnMessage(kChannelNameHash, send_pb_msg, msg_info),
               "i not supported.");
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo

