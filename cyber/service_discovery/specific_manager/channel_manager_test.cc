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

#include "cyber/service_discovery/specific_manager/channel_manager.h"

#include <memory>
#include <string>
#include <vector>
#include "gtest/gtest.h"

#include "cyber/common/global_data.h"
#include "cyber/message/message_traits.h"
#include "cyber/message/protobuf_factory.h"
#include "cyber/proto/unit_test.pb.h"
#include "cyber/transport/common/identity.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class ChannelManagerTest : public ::testing::Test {
 protected:
  ChannelManagerTest() : channel_num_(10) {
    RoleAttributes role_attr;
    role_attr.set_host_name(common::GlobalData::Instance()->HostName());
    role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());

    // add writers
    for (int i = 0; i < channel_num_; ++i) {
      role_attr.set_node_name("node_" + std::to_string(i));
      uint64_t node_id =
          common::GlobalData::RegisterNode(role_attr.node_name());
      role_attr.set_node_id(node_id);
      role_attr.set_channel_name("channel_" + std::to_string(i));
      uint64_t channel_id = common::GlobalData::Instance()->RegisterChannel(
          role_attr.channel_name());
      role_attr.set_channel_id(channel_id);
      transport::Identity id;
      role_attr.set_id(id.HashValue());
      channel_manager_.Join(role_attr, RoleType::ROLE_WRITER);
    }

    // add readers
    for (int i = 0; i < channel_num_; ++i) {
      role_attr.set_node_name("node_" + std::to_string(i));
      uint64_t node_id =
          common::GlobalData::RegisterNode(role_attr.node_name());
      role_attr.set_node_id(node_id);
      role_attr.set_channel_name("channel_" + std::to_string(i));
      uint64_t channel_id = common::GlobalData::Instance()->RegisterChannel(
          role_attr.channel_name());
      role_attr.set_channel_id(channel_id);
      transport::Identity id;
      role_attr.set_id(id.HashValue());
      channel_manager_.Join(role_attr, RoleType::ROLE_READER);
    }
  }
  virtual ~ChannelManagerTest() { channel_manager_.Shutdown(); }

  virtual void SetUp() {}

  virtual void TearDown() {}

  int channel_num_;
  ChannelManager channel_manager_;
};

TEST_F(ChannelManagerTest, get_channel_names) {
  std::vector<std::string> channels;
  EXPECT_TRUE(channels.empty());

  channel_manager_.GetChannelNames(&channels);
  EXPECT_EQ(channels.size(), channel_num_);
}

TEST_F(ChannelManagerTest, get_proto_desc) {
  const std::string guard = "guard";
  std::string proto_desc(guard);

  // channel does not exist
  channel_manager_.GetProtoDesc("wasd", &proto_desc);
  EXPECT_EQ(proto_desc, guard);

  // channel exists, but no proto desc
  channel_manager_.GetProtoDesc("channel_0", &proto_desc);
  EXPECT_EQ(proto_desc, guard);

  // add a writer with empty proto desc
  RoleAttributes role_attr;
  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("proto");
  uint64_t node_id = common::GlobalData::RegisterNode("proto");
  role_attr.set_node_id(node_id);
  role_attr.set_channel_name("wasd");
  uint64_t channel_id = common::GlobalData::Instance()->RegisterChannel("wasd");
  role_attr.set_channel_id(channel_id);
  transport::Identity id_0;
  role_attr.set_id(id_0.HashValue());
  role_attr.set_proto_desc("");
  EXPECT_FALSE(channel_manager_.Join(role_attr, RoleType::ROLE_WRITER));

  channel_manager_.GetProtoDesc("wasd", &proto_desc);
  EXPECT_EQ(proto_desc, "");

  EXPECT_FALSE(channel_manager_.Leave(role_attr, RoleType::ROLE_WRITER));

  // add a writer with real proto desc
  role_attr.set_node_name("proto");
  node_id = common::GlobalData::RegisterNode("proto");
  role_attr.set_node_id(node_id);
  role_attr.set_channel_name("jkl");
  channel_id = common::GlobalData::Instance()->RegisterChannel("jkl");
  role_attr.set_channel_id(channel_id);
  transport::Identity id_1;
  role_attr.set_id(id_1.HashValue());
  std::string tmp("");
  message::GetDescriptorString<proto::Chatter>(
      message::MessageType<proto::Chatter>(), &tmp);
  role_attr.set_proto_desc(tmp);
  EXPECT_FALSE(channel_manager_.Join(role_attr, RoleType::ROLE_WRITER));

  proto_desc = guard;
  EXPECT_TRUE(channel_manager_.HasWriter("jkl"));
  channel_manager_.GetProtoDesc("jkl", &proto_desc);
  EXPECT_NE(proto_desc, guard);

  EXPECT_FALSE(channel_manager_.Leave(role_attr, RoleType::ROLE_WRITER));
}

TEST_F(ChannelManagerTest, has_writer) {
  for (int i = 0; i < channel_num_; ++i) {
    EXPECT_TRUE(channel_manager_.HasWriter("channel_" + std::to_string(i)));
  }
}

TEST_F(ChannelManagerTest, get_writers_attr) {
  std::vector<proto::RoleAttributes> writers;
  EXPECT_TRUE(writers.empty());

  channel_manager_.GetWriters(&writers);
  EXPECT_EQ(writers.size(), channel_num_);

  writers.clear();
  for (int i = 0; i < channel_num_; ++i) {
    channel_manager_.GetWritersOfChannel("channel_" + std::to_string(i),
                                         &writers);
    EXPECT_EQ(writers.size(), 1);
    writers.clear();
    channel_manager_.GetWritersOfNode("node_" + std::to_string(i), &writers);
    EXPECT_EQ(writers.size(), 1);
    writers.clear();
  }

  RoleAttributes role_attr;
  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("node_extra");
  uint64_t node_id = common::GlobalData::RegisterNode("node_extra");
  role_attr.set_node_id(node_id);

  // add writers
  for (int i = 0; i < 100; ++i) {
    role_attr.set_channel_name("channel_" + std::to_string(i));
    uint64_t channel_id = common::GlobalData::Instance()->RegisterChannel(
        role_attr.channel_name());
    role_attr.set_channel_id(channel_id);
    transport::Identity id;
    role_attr.set_id(id.HashValue());
    channel_manager_.Join(role_attr, RoleType::ROLE_WRITER);
  }

  writers.clear();
  channel_manager_.GetWritersOfNode("node_extra", &writers);
  EXPECT_EQ(writers.size(), 100);
}

TEST_F(ChannelManagerTest, has_reader) {
  for (int i = 0; i < channel_num_; ++i) {
    EXPECT_TRUE(channel_manager_.HasReader("channel_" + std::to_string(i)));
  }
}

TEST_F(ChannelManagerTest, get_readers_attr) {
  std::vector<proto::RoleAttributes> readers;
  EXPECT_TRUE(readers.empty());

  channel_manager_.GetReaders(&readers);
  EXPECT_EQ(readers.size(), channel_num_);

  readers.clear();
  for (int i = 0; i < channel_num_; ++i) {
    channel_manager_.GetReadersOfChannel("channel_" + std::to_string(i),
                                         &readers);
    EXPECT_EQ(readers.size(), 1);
    readers.clear();
    channel_manager_.GetReadersOfNode("node_" + std::to_string(i), &readers);
    EXPECT_EQ(readers.size(), 1);
    readers.clear();
  }

  channel_manager_.GetReadersOfChannel("channel_0", &readers);
  EXPECT_EQ(readers.size(), 1);

  RoleAttributes role_attr(readers[0]);
  EXPECT_FALSE(channel_manager_.Leave(role_attr, RoleType::ROLE_READER));

  readers.clear();
  channel_manager_.GetReadersOfChannel("channel_0", &readers);
  EXPECT_EQ(readers.size(), 0);

  EXPECT_FALSE(channel_manager_.Join(role_attr, RoleType::ROLE_READER));
  readers.clear();
  channel_manager_.GetReadersOfChannel("channel_0", &readers);
  EXPECT_EQ(readers.size(), 1);

  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("node_extra");
  uint64_t node_id = common::GlobalData::RegisterNode("node_extra");
  role_attr.set_node_id(node_id);

  // add readers
  for (int i = 0; i < 100; ++i) {
    role_attr.set_channel_name("channel_" + std::to_string(i));
    uint64_t channel_id = common::GlobalData::Instance()->RegisterChannel(
        role_attr.channel_name());
    role_attr.set_channel_id(channel_id);
    transport::Identity id;
    role_attr.set_id(id.HashValue());
    channel_manager_.Join(role_attr, RoleType::ROLE_READER);
  }

  readers.clear();
  channel_manager_.GetReadersOfNode("node_extra", &readers);
  EXPECT_EQ(readers.size(), 100);
}

TEST_F(ChannelManagerTest, change) {
  RoleAttributes role_attr;
  EXPECT_FALSE(channel_manager_.Join(role_attr, RoleType::ROLE_NODE));
  EXPECT_FALSE(channel_manager_.Join(role_attr, RoleType::ROLE_WRITER));

  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("add_change");
  uint64_t node_id = common::GlobalData::RegisterNode("add_change");
  role_attr.set_node_id(node_id);
  role_attr.set_channel_name("wasd");
  uint64_t channel_id = common::GlobalData::Instance()->RegisterChannel("wasd");
  role_attr.set_channel_id(channel_id);
  EXPECT_FALSE(channel_manager_.Join(role_attr, RoleType::ROLE_WRITER));

  transport::Identity id;
  role_attr.set_id(id.HashValue());
  EXPECT_FALSE(channel_manager_.Join(role_attr, RoleType::ROLE_WRITER));

  EXPECT_TRUE(channel_manager_.HasWriter("channel_0"));
}

TEST_F(ChannelManagerTest, get_upstream_downstream) {
  std::vector<proto::RoleAttributes> nodes;
  for (int i = 0; i < channel_num_; ++i) {
    channel_manager_.GetUpstreamOfNode("node_" + std::to_string(i), &nodes);
    EXPECT_EQ(nodes.size(), 1);
    nodes.clear();
    channel_manager_.GetDownstreamOfNode("node_" + std::to_string(i), &nodes);
    EXPECT_EQ(nodes.size(), 1);
    nodes.clear();
  }

  // add dag like this
  // A---a---B---c---D
  // |               |
  // ----b---C---d----
  RoleAttributes role_attr;
  role_attr.set_host_name(common::GlobalData::Instance()->HostName());
  role_attr.set_process_id(common::GlobalData::Instance()->ProcessId());
  role_attr.set_node_name("A");
  uint64_t node_id = common::GlobalData::RegisterNode(role_attr.node_name());
  role_attr.set_node_id(node_id);
  role_attr.set_channel_name("a");
  uint64_t channel_id =
      common::GlobalData::Instance()->RegisterChannel(role_attr.channel_name());
  role_attr.set_channel_id(channel_id);
  role_attr.set_id(transport::Identity().HashValue());
  channel_manager_.Join(role_attr, RoleType::ROLE_WRITER);

  role_attr.set_channel_name("b");
  channel_id =
      common::GlobalData::Instance()->RegisterChannel(role_attr.channel_name());
  role_attr.set_channel_id(channel_id);
  role_attr.set_id(transport::Identity().HashValue());
  channel_manager_.Join(role_attr, RoleType::ROLE_WRITER);

  role_attr.set_node_name("B");
  node_id = common::GlobalData::RegisterNode(role_attr.node_name());
  role_attr.set_node_id(node_id);
  role_attr.set_channel_name("c");
  channel_id =
      common::GlobalData::Instance()->RegisterChannel(role_attr.channel_name());
  role_attr.set_channel_id(channel_id);
  role_attr.set_id(transport::Identity().HashValue());
  channel_manager_.Join(role_attr, RoleType::ROLE_WRITER);

  role_attr.set_channel_name("a");
  channel_id =
      common::GlobalData::Instance()->RegisterChannel(role_attr.channel_name());
  role_attr.set_channel_id(channel_id);
  role_attr.set_id(transport::Identity().HashValue());
  channel_manager_.Join(role_attr, RoleType::ROLE_READER);

  role_attr.set_node_name("C");
  node_id = common::GlobalData::RegisterNode(role_attr.node_name());
  role_attr.set_node_id(node_id);
  role_attr.set_channel_name("b");
  channel_id =
      common::GlobalData::Instance()->RegisterChannel(role_attr.channel_name());
  role_attr.set_channel_id(channel_id);
  role_attr.set_id(transport::Identity().HashValue());
  channel_manager_.Join(role_attr, RoleType::ROLE_READER);

  role_attr.set_channel_name("d");
  channel_id =
      common::GlobalData::Instance()->RegisterChannel(role_attr.channel_name());
  role_attr.set_channel_id(channel_id);
  role_attr.set_id(transport::Identity().HashValue());
  channel_manager_.Join(role_attr, RoleType::ROLE_WRITER);

  role_attr.set_node_name("D");
  node_id = common::GlobalData::RegisterNode(role_attr.node_name());
  role_attr.set_node_id(node_id);
  role_attr.set_channel_name("c");
  channel_id =
      common::GlobalData::Instance()->RegisterChannel(role_attr.channel_name());
  role_attr.set_channel_id(channel_id);
  role_attr.set_id(transport::Identity().HashValue());
  channel_manager_.Join(role_attr, RoleType::ROLE_READER);

  role_attr.set_channel_name("d");
  channel_id =
      common::GlobalData::Instance()->RegisterChannel(role_attr.channel_name());
  role_attr.set_channel_id(channel_id);
  role_attr.set_id(transport::Identity().HashValue());
  channel_manager_.Join(role_attr, RoleType::ROLE_READER);

  nodes.clear();
  channel_manager_.GetUpstreamOfNode("A", &nodes);
  EXPECT_TRUE(nodes.empty());

  nodes.clear();
  channel_manager_.GetDownstreamOfNode("A", &nodes);
  EXPECT_EQ(nodes.size(), 2);

  nodes.clear();
  channel_manager_.GetUpstreamOfNode("B", &nodes);
  EXPECT_EQ(nodes.size(), 1);

  nodes.clear();
  channel_manager_.GetDownstreamOfNode("B", &nodes);
  EXPECT_EQ(nodes.size(), 1);

  nodes.clear();
  channel_manager_.GetUpstreamOfNode("C", &nodes);
  EXPECT_EQ(nodes.size(), 1);

  nodes.clear();
  channel_manager_.GetDownstreamOfNode("C", &nodes);
  EXPECT_EQ(nodes.size(), 1);

  nodes.clear();
  channel_manager_.GetUpstreamOfNode("D", &nodes);
  EXPECT_EQ(nodes.size(), 2);

  nodes.clear();
  channel_manager_.GetDownstreamOfNode("D", &nodes);
  EXPECT_TRUE(nodes.empty());

  nodes.clear();
  channel_manager_.GetUpstreamOfNode("E", &nodes);
  EXPECT_TRUE(nodes.empty());

  nodes.clear();
  channel_manager_.GetDownstreamOfNode("E", &nodes);
  EXPECT_TRUE(nodes.empty());

  EXPECT_EQ(channel_manager_.GetFlowDirection("A", "B"), UPSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("A", "C"), UPSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("A", "D"), UPSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("B", "D"), UPSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("C", "D"), UPSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("B", "A"), DOWNSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("C", "A"), DOWNSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("D", "A"), DOWNSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("D", "B"), DOWNSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("D", "C"), DOWNSTREAM);
  EXPECT_EQ(channel_manager_.GetFlowDirection("A", "E"), UNREACHABLE);
  EXPECT_EQ(channel_manager_.GetFlowDirection("E", "A"), UNREACHABLE);
}

TEST_F(ChannelManagerTest, is_message_type_matching) {
  const std::string raw_msg_type_1 =
      message::MessageType<message::RawMessage>();
  const std::string py_msg_type =
      message::MessageType<message::PyMessageWrap>();
  const std::string chatter_msg_type = message::MessageType<proto::Chatter>();
  const std::string change_msg_type = message::MessageType<proto::ChangeMsg>();
  EXPECT_TRUE(channel_manager_.IsMessageTypeMatching(chatter_msg_type,
                                                     chatter_msg_type));
  EXPECT_FALSE(channel_manager_.IsMessageTypeMatching(chatter_msg_type,
                                                      change_msg_type));
  EXPECT_TRUE(
      channel_manager_.IsMessageTypeMatching(chatter_msg_type, raw_msg_type_1));
  EXPECT_TRUE(
      channel_manager_.IsMessageTypeMatching(chatter_msg_type, py_msg_type));
  EXPECT_TRUE(
      channel_manager_.IsMessageTypeMatching(raw_msg_type_1, py_msg_type));
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
