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

#include <memory>
#include "gtest/gtest.h"

#include "cyber/common/util.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/transport/common/identity.h"

namespace apollo {
namespace cyber {
namespace transport {

TEST(DispatcherTest, on_message) {
  auto dispatcher = IntraDispatcher::Instance();
  std::vector<std::shared_ptr<proto::Chatter>> chatter_msgs;
  std::vector<std::shared_ptr<message::RawMessage>> raw_msgs;
  auto chatter = std::make_shared<proto::Chatter>();
  chatter->set_content("chatter");
  auto chatter2 = std::make_shared<proto::Chatter>();
  chatter2->set_content("raw message");
  std::string str;
  chatter2->SerializeToString(&str);
  auto raw = std::make_shared<message::RawMessage>(str);
  auto chatter_callback = [&chatter_msgs](
                              const std::shared_ptr<proto::Chatter>& msg,
                              const MessageInfo&) {
    AINFO << "chatter callback";
    chatter_msgs.push_back(msg);
  };
  auto raw_callback = [&raw_msgs](
                          const std::shared_ptr<message::RawMessage>& msg,
                          const MessageInfo&) {
    AINFO << "raw callback";
    raw_msgs.push_back(msg);
  };
  MessageInfo msg_info;

  const std::string channel_name = "channel";
  const uint64_t channel_id = common::Hash(channel_name);
  proto::RoleAttributes self_attr1;
  self_attr1.set_channel_name(channel_name);
  self_attr1.set_channel_id(channel_id);
  self_attr1.set_id(Identity().HashValue());
  proto::RoleAttributes self_attr2(self_attr1);
  self_attr2.set_id(Identity().HashValue());
  proto::RoleAttributes self_none;
  self_none.set_channel_name("channel1");
  self_none.set_channel_id(common::Hash("channel1"));
  proto::RoleAttributes oppo_attr1(self_attr1);
  Identity identity1;
  oppo_attr1.set_id(identity1.HashValue());
  proto::RoleAttributes oppo_attr2(self_attr1);
  Identity identity2;
  oppo_attr2.set_id(identity2.HashValue());

  // AddListener
  // add chatter
  dispatcher->AddListener<proto::Chatter>(self_attr1, chatter_callback);
  // add raw
  dispatcher->AddListener<message::RawMessage>(self_attr2, raw_callback);
  // add chatter opposite
  dispatcher->AddListener<proto::Chatter>(self_attr1, oppo_attr1,
                                          chatter_callback);
  // add raw opposite
  dispatcher->AddListener<message::RawMessage>(self_attr2, oppo_attr1,
                                               raw_callback);

  // run 1 + 2
  dispatcher->OnMessage<proto::Chatter>(channel_id, chatter, msg_info);
  EXPECT_EQ(1, chatter_msgs.size());
  EXPECT_EQ(1, raw_msgs.size());

  // run 1, 3 + 2, 4
  msg_info.set_sender_id(identity1);
  dispatcher->OnMessage<message::RawMessage>(channel_id, raw, msg_info);
  EXPECT_EQ(3, chatter_msgs.size());
  EXPECT_EQ(3, raw_msgs.size());
  Identity identity3;
  msg_info.set_sender_id(identity3);
  // no oppo handler, but self handler will run
  dispatcher->OnMessage<proto::Chatter>(channel_id, chatter, msg_info);
  // run 1 + 2
  EXPECT_EQ(4, chatter_msgs.size());
  EXPECT_EQ(4, raw_msgs.size());

  // RemoveListenerHandler
  // find no key
  dispatcher->RemoveListener<proto::Chatter>(self_none);
  dispatcher->RemoveListener<proto::Chatter>(self_none, oppo_attr2);
  // find keys
  dispatcher->RemoveListener<proto::Chatter>(self_attr1);
  dispatcher->RemoveListener<proto::Chatter>(self_attr2);
  dispatcher->RemoveListener<proto::Chatter>(self_attr1, oppo_attr1);
  dispatcher->RemoveListener<proto::Chatter>(self_attr2, oppo_attr2);

  // run nothing
  raw_msgs.clear();
  chatter_msgs.clear();
  MessageInfo msg_info1;
  dispatcher->OnMessage<proto::Chatter>(channel_id, chatter, msg_info);
  EXPECT_EQ(0, chatter_msgs.size());
  EXPECT_EQ(0, raw_msgs.size());
  msg_info.set_sender_id(identity1);
  dispatcher->OnMessage<proto::Chatter>(channel_id, chatter, msg_info);
  EXPECT_EQ(0, chatter_msgs.size());
  EXPECT_EQ(0, raw_msgs.size());
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
