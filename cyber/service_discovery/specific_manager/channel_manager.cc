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
#include "cyber/init.h"
#include "cyber/message/message_traits.h"
#include "cyber/message/py_message.h"
#include "cyber/message/raw_message.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

ChannelManager::ChannelManager() {
  allowed_role_ |= 1 << RoleType::ROLE_WRITER;
  allowed_role_ |= 1 << RoleType::ROLE_READER;
  change_type_ = ChangeType::CHANGE_CHANNEL;
  channel_name_ = "channel_change_broadcast";

  auto& global_conf = common::GlobalData::Instance()->Config();
  if (global_conf.has_topology_conf() &&
      global_conf.topology_conf().has_channel_conf()) {
    auto& channel_conf = global_conf.topology_conf().channel_conf();
    for (auto& channel : channel_conf.intra_channels()) {
      ADEBUG << "intra channel: " << channel;
      intra_channels_.emplace(channel);
    }
  }
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

bool ChannelManager::NeedPublish(const ChangeMsg& msg) const {
  auto& channel = msg.role_attr().channel_name();
  if (intra_channels_.count(channel) > 0) {
    return false;
  }
  return true;
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
  auto role = std::make_shared<RoleBase>(msg.role_attr(), msg.timestamp());
  uint64_t key = role->attributes().channel_id();
  if (!channel_delegates_.Add(key, role, false)) {
    RolePtr existing_delegate;
    if (!channel_delegates_.Search(key, &existing_delegate)) {
      channel_delegates_.Add(key, role);
    } else {
      auto& exist_msg_type = existing_delegate->attributes().message_type();
      auto& join_msg_type = role->attributes().message_type();
      const std::string RAW_MESSAGE_TYPE =
          message::MessageType<message::RawMessage>();
      const std::string PY_MESSAGE_TYPE =
          message::MessageType<message::PyMessageWrap>();
      if ((exist_msg_type != RAW_MESSAGE_TYPE &&
           join_msg_type != RAW_MESSAGE_TYPE &&
           exist_msg_type != join_msg_type) &&
          (exist_msg_type != join_msg_type &&
           exist_msg_type != PY_MESSAGE_TYPE &&
           join_msg_type != PY_MESSAGE_TYPE)) {
        auto newer = role;
        auto older = existing_delegate;
        if (!older->IsEarlierThan(*newer)) {
          newer = existing_delegate;
          older = role;
          channel_delegates_.Add(key, older);
        }

        if (newer->attributes().process_id() == process_id_ &&
            newer->attributes().host_name() == host_name_) {
          AERROR << "this process will be terminated due to mismatch message "
                    "type of channel["
                 << newer->attributes().channel_name() << "], satisfied["
                 << older->attributes().message_type() << "] given["
                 << newer->attributes().message_type() << "].";
          AsyncShutdown();
          return;
        }
      }
    }
  }

  Vertice v(msg.role_attr().node_name());
  Edge e;
  e.set_value(msg.role_attr().channel_name());
  if (msg.role_type() == RoleType::ROLE_WRITER) {
    if (msg.role_attr().has_proto_desc() &&
        msg.role_attr().proto_desc() != "") {
      message::ProtobufFactory::Instance()->RegisterMessage(
          msg.role_attr().proto_desc());
    }

    node_writers_.Add(role->attributes().node_id(), role);
    channel_writers_.Add(role->attributes().channel_id(), role);
    e.set_src(v);
  } else {
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

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
