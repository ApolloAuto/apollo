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

TEST(IntraDispatcherTest, on_message) {
  auto dispatcher = IntraDispatcher::Instance();

  auto send_pb_msg = std::make_shared<proto::Chatter>();
  send_pb_msg->set_timestamp(123);
  send_pb_msg->set_lidar_timestamp(456);
  send_pb_msg->set_seq(789);
  send_pb_msg->set_content("on_message");

  MessageInfo msg_info;

  dispatcher->OnMessage(common::Hash("pb_channel"), send_pb_msg, msg_info);

  RoleAttributes attr;
  attr.set_channel_name("pb_channel");
  attr.set_channel_id(common::Hash("pb_channel"));
  Identity pb_id;
  attr.set_id(pb_id.HashValue());
  auto recv_pb_msg = std::make_shared<proto::Chatter>();
  dispatcher->AddListener<proto::Chatter>(
      attr, [&recv_pb_msg](const std::shared_ptr<proto::Chatter>& msg,
                           const MessageInfo& msg_info) {
        (void)msg_info;
        recv_pb_msg->CopyFrom(*msg);
      });
  dispatcher->OnMessage(common::Hash("pb_channel"), send_pb_msg, msg_info);

  EXPECT_EQ(recv_pb_msg->timestamp(), 123);
  EXPECT_EQ(recv_pb_msg->lidar_timestamp(), 456);
  EXPECT_EQ(recv_pb_msg->seq(), 789);
  EXPECT_EQ(recv_pb_msg->content(), "on_message");

  attr.set_channel_name("raw_channel");
  attr.set_channel_id(common::Hash("raw_channel"));
  Identity raw_id;
  attr.set_id(raw_id.HashValue());
  auto recv_raw_msg = std::make_shared<message::RawMessage>();
  dispatcher->AddListener<message::RawMessage>(
      attr, [&recv_raw_msg](const std::shared_ptr<message::RawMessage>& msg,
                            const MessageInfo& msg_info) {
        (void)msg_info;
        recv_raw_msg->message = msg->message;
      });

  auto send_raw_msg = std::make_shared<message::RawMessage>("on_message");
  dispatcher->OnMessage(common::Hash("raw_channel"), send_raw_msg, msg_info);

  EXPECT_EQ(recv_raw_msg->message, send_raw_msg->message);
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
