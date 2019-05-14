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

#include <algorithm>
#include <set>
#include <utility>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/message/message_traits.h"
#include "cyber/message/py_message.h"
#include "cyber/message/raw_message.h"
#include "cyber/state.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

ChannelManager::ChannelManager() {
  allowed_role_ |= 1 << RoleType::ROLE_WRITER;
  allowed_role_ |= 1 << RoleType::ROLE_READER;
  change_type_ = ChangeType::CHANGE_CHANNEL;
  channel_name_ = "channel_change_broadcast";
  exempted_msg_types_.emplace(message::MessageType<message::RawMessage>());
  exempted_msg_types_.emplace(message::MessageType<message::PyMessageWrap>());
}

ChannelManager::~ChannelManager() {}

void ChannelManager::GetChannelNames(std::vector<std::string>* channels) {
  RETURN_IF_NULL(channels);

  std::unordered_set<std::string> local_channels;
  std::vector<RolePtr> roles;
  channel_writers_.GetAllRoles(&roles);
  channel_readers_.GetAllRoles(&roles);
  for (auto& role : roles) {
    local_channels.emplace(role->attributes().channel_name());
  }
  std::move(local_channels.begin(), local_channels.end(),
            std::back_inserter(*channels));
}

void ChannelManager::GetProtoDesc(const std::string& channel_name,
                                  std::string* proto_desc) {
  RETURN_IF_NULL(proto_desc);
  uint64_t key = common::GlobalData::RegisterChannel(channel_name);
  RolePtr writer = nullptr;
  if (!channel_writers_.Search(key, &writer)) {
    return;
  }

  if (writer->attributes().has_proto_desc()) {
    *proto_desc = writer->attributes().proto_desc();
  }
}

void ChannelManager::GetMsgType(const std::string& channel_name,
                                std::string* msg_type) {
  RETURN_IF_NULL(msg_type);
  uint64_t key = common::GlobalData::RegisterChannel(channel_name);
  RolePtr writer = nullptr;
  if (!channel_writers_.Search(key, &writer)) {
    AERROR << "cannot serarch writer of channel: " << channel_name
           << " key: " << key;
    return;
  }

  if (writer->attributes().has_message_type()) {
    *msg_type = writer->attributes().message_type();
  }
}

bool ChannelManager::HasWriter(const std::string& channel_name) {
  uint64_t key = common::GlobalData::RegisterChannel(channel_name);
  return channel_writers_.Search(key);
}

void ChannelManager::GetWriters(RoleAttrVec* writers) {
  RETURN_IF_NULL(writers);
  channel_writers_.GetAllRoles(writers);
}

void ChannelManager::GetWritersOfNode(const std::string& node_name,
                                      RoleAttrVec* writers) {
  RETURN_IF_NULL(writers);
  uint64_t key = common::GlobalData::RegisterNode(node_name);
  node_writers_.Search(key, writers);
}

void ChannelManager::GetWritersOfChannel(const std::string& channel_name,
                                         RoleAttrVec* writers) {
  RETURN_IF_NULL(writers);
  uint64_t key = common::GlobalData::RegisterChannel(channel_name);
  channel_writers_.Search(key, writers);
}

bool ChannelManager::HasReader(const std::string& channel_name) {
  uint64_t key = common::GlobalData::RegisterChannel(channel_name);
  return channel_readers_.Search(key);
}

void ChannelManager::GetReaders(RoleAttrVec* readers) {
  RETURN_IF_NULL(readers);
  channel_readers_.GetAllRoles(readers);
}

void ChannelManager::GetReadersOfNode(const std::string& node_name,
                                      RoleAttrVec* readers) {
  RETURN_IF_NULL(readers);
  uint64_t key = common::GlobalData::RegisterNode(node_name);
  node_readers_.Search(key, readers);
}

void ChannelManager::GetReadersOfChannel(const std::string& channel_name,
                                         RoleAttrVec* readers) {
  RETURN_IF_NULL(readers);
  uint64_t key = common::GlobalData::RegisterChannel(channel_name);
  channel_readers_.Search(key, readers);
}

void ChannelManager::GetUpstreamOfNode(const std::string& node_name,
                                       RoleAttrVec* upstream_nodes) {
  RETURN_IF_NULL(upstream_nodes);

  RoleAttrVec readers;
  GetReadersOfNode(node_name, &readers);
  if (readers.empty()) {
    return;
  }
  std::unordered_set<std::string> channels;
  for (auto& reader : readers) {
    channels.emplace(reader.channel_name());
  }

  RoleAttrVec writers;
  for (auto& channel : channels) {
    GetWritersOfChannel(channel, &writers);
  }

  std::unordered_map<std::string, proto::RoleAttributes> nodes;
  for (auto& writer : writers) {
    proto::RoleAttributes attr;
    attr.set_host_name(writer.host_name());
    attr.set_process_id(writer.process_id());
    attr.set_node_name(writer.node_name());
    attr.set_node_id(writer.node_id());
    nodes[attr.node_name()] = attr;
  }
  for (auto& item : nodes) {
    upstream_nodes->emplace_back(item.second);
  }
}

void ChannelManager::GetDownstreamOfNode(const std::string& node_name,
                                         RoleAttrVec* downstream_nodes) {
  RETURN_IF_NULL(downstream_nodes);

  RoleAttrVec writers;
  GetWritersOfNode(node_name, &writers);
  if (writers.empty()) {
    return;
  }
  std::unordered_set<std::string> channels;
  for (auto& writer : writers) {
    channels.emplace(writer.channel_name());
  }

  RoleAttrVec readers;
  for (auto& channel : channels) {
    GetReadersOfChannel(channel, &readers);
  }

  std::unordered_map<std::string, proto::RoleAttributes> nodes;
  for (auto& reader : readers) {
    proto::RoleAttributes attr;
    attr.set_host_name(reader.host_name());
    attr.set_process_id(reader.process_id());
    attr.set_node_name(reader.node_name());
    attr.set_node_id(reader.node_id());
    nodes[attr.node_name()] = attr;
  }
  for (auto& item : nodes) {
    downstream_nodes->emplace_back(item.second);
  }
}

FlowDirection ChannelManager::GetFlowDirection(
    const std::string& lhs_node_name, const std::string& rhs_node_name) {
  Vertice lhs(lhs_node_name);
  Vertice rhs(rhs_node_name);
  return node_graph_.GetDirectionOf(lhs, rhs);
}

bool ChannelManager::IsMessageTypeMatching(const std::string& lhs,
                                           const std::string& rhs) {
  if (lhs == rhs) {
    return true;
  }
  if (exempted_msg_types_.count(lhs) > 0) {
    return true;
  }
  if (exempted_msg_types_.count(rhs) > 0) {
    return true;
  }
  return false;
}

bool ChannelManager::Check(const RoleAttributes& attr) {
  RETURN_VAL_IF(!attr.has_channel_name(), false);
  RETURN_VAL_IF(!attr.has_channel_id(), false);
  RETURN_VAL_IF(!attr.has_id(), false);
  return true;
}

void ChannelManager::Dispose(const ChangeMsg& msg) {
  if (msg.operate_type() == OperateType::OPT_JOIN) {
    DisposeJoin(msg);
  } else {
    DisposeLeave(msg);
  }
  Notify(msg);
}

void ChannelManager::OnTopoModuleLeave(const std::string& host_name,
                                       int process_id) {
  RETURN_IF(!is_discovery_started_.load());

  RoleAttributes attr;
  attr.set_host_name(host_name);
  attr.set_process_id(process_id);

  std::vector<RolePtr> writers_to_remove;
  channel_writers_.Search(attr, &writers_to_remove);

  std::vector<RolePtr> readers_to_remove;
  channel_readers_.Search(attr, &readers_to_remove);

  ChangeMsg msg;
  for (auto& writer : writers_to_remove) {
    Convert(writer->attributes(), RoleType::ROLE_WRITER, OperateType::OPT_LEAVE,
            &msg);
    DisposeLeave(msg);
    Notify(msg);
  }

  for (auto& reader : readers_to_remove) {
    Convert(reader->attributes(), RoleType::ROLE_READER, OperateType::OPT_LEAVE,
            &msg);
    DisposeLeave(msg);
    Notify(msg);
  }
}

void ChannelManager::DisposeJoin(const ChangeMsg& msg) {
  ScanMessageType(msg);

  Vertice v(msg.role_attr().node_name());
  Edge e;
  e.set_value(msg.role_attr().channel_name());
  if (msg.role_type() == RoleType::ROLE_WRITER) {
    if (msg.role_attr().has_proto_desc() &&
        msg.role_attr().proto_desc() != "") {
      message::ProtobufFactory::Instance()->RegisterMessage(
          msg.role_attr().proto_desc());
    }
    auto role = std::make_shared<RoleWriter>(msg.role_attr(), msg.timestamp());
    node_writers_.Add(role->attributes().node_id(), role);
    channel_writers_.Add(role->attributes().channel_id(), role);
    e.set_src(v);
  } else {
    auto role = std::make_shared<RoleReader>(msg.role_attr(), msg.timestamp());
    node_readers_.Add(role->attributes().node_id(), role);
    channel_readers_.Add(role->attributes().channel_id(), role);
    e.set_dst(v);
  }
  node_graph_.Insert(e);
}

void ChannelManager::DisposeLeave(const ChangeMsg& msg) {
  Vertice v(msg.role_attr().node_name());
  Edge e;
  e.set_value(msg.role_attr().channel_name());
  if (msg.role_type() == RoleType::ROLE_WRITER) {
    auto role = std::make_shared<RoleWriter>(msg.role_attr());
    node_writers_.Remove(role->attributes().node_id(), role);
    channel_writers_.Remove(role->attributes().channel_id(), role);
    e.set_src(v);
  } else {
    auto role = std::make_shared<RoleReader>(msg.role_attr());
    node_readers_.Remove(role->attributes().node_id(), role);
    channel_readers_.Remove(role->attributes().channel_id(), role);
    e.set_dst(v);
  }
  node_graph_.Delete(e);
}

void ChannelManager::ScanMessageType(const ChangeMsg& msg) {
  uint64_t key = msg.role_attr().channel_id();
  std::string role_type("reader");
  if (msg.role_type() == RoleType::ROLE_WRITER) {
    role_type = "writer";
  }

  RoleAttrVec existed_writers;
  channel_writers_.Search(key, &existed_writers);
  for (auto& w_attr : existed_writers) {
    if (!IsMessageTypeMatching(msg.role_attr().message_type(),
                               w_attr.message_type())) {
      AERROR << "newly added " << role_type << "(belongs to node["
             << msg.role_attr().node_name() << "])"
             << "'s message type[" << msg.role_attr().message_type()
             << "] does not match the exsited writer(belongs to node["
             << w_attr.node_name() << "])'s message type["
             << w_attr.message_type() << "].";
    }
  }

  RoleAttrVec existed_readers;
  channel_readers_.Search(key, &existed_readers);
  for (auto& r_attr : existed_readers) {
    if (!IsMessageTypeMatching(msg.role_attr().message_type(),
                               r_attr.message_type())) {
      AERROR << "newly added " << role_type << "(belongs to node["
             << msg.role_attr().node_name() << "])"
             << "'s message type[" << msg.role_attr().message_type()
             << "] does not match the exsited reader(belongs to node["
             << r_attr.node_name() << "])'s message type["
             << r_attr.message_type() << "].";
    }
  }
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
