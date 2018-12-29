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

#ifndef CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_CHANNEL_MANAGER_H_
#define CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_CHANNEL_MANAGER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cyber/service_discovery/container/graph.h"
#include "cyber/service_discovery/container/multi_value_warehouse.h"
#include "cyber/service_discovery/container/single_value_warehouse.h"
#include "cyber/service_discovery/role/role.h"
#include "cyber/service_discovery/specific_manager/manager.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class TopologyManager;

class ChannelManager : public Manager {
  friend class TopologyManager;

 public:
  using RoleAttrVec = std::vector<proto::RoleAttributes>;
  using WriterWarehouse = MultiValueWarehouse;
  using ReaderWarehouse = MultiValueWarehouse;
  using ExemptedMessageTypes = std::unordered_set<std::string>;

  ChannelManager();
  virtual ~ChannelManager();

  void GetChannelNames(std::vector<std::string>* channels);
  void GetProtoDesc(const std::string& channel_name, std::string* proto_desc);

  bool HasWriter(const std::string& channel_name);
  void GetWriters(RoleAttrVec* writers);
  void GetWritersOfNode(const std::string& node_name, RoleAttrVec* writers);
  void GetWritersOfChannel(const std::string& channel_name,
                           RoleAttrVec* writers);

  bool HasReader(const std::string& channel_name);
  void GetReaders(RoleAttrVec* readers);
  void GetReadersOfNode(const std::string& node_name, RoleAttrVec* readers);
  void GetReadersOfChannel(const std::string& channel_name,
                           RoleAttrVec* readers);

  void GetUpstreamOfNode(const std::string& node_name,
                         RoleAttrVec* upstream_nodes);
  void GetDownstreamOfNode(const std::string& node_name,
                           RoleAttrVec* downstream_nodes);

  FlowDirection GetFlowDirection(const std::string& lhs_node_name,
                                 const std::string& rhs_node_name);

  bool IsMessageTypeMatching(const std::string& lhs, const std::string& rhs);

 private:
  bool Check(const RoleAttributes& attr) override;
  void Dispose(const ChangeMsg& msg) override;
  void OnTopoModuleLeave(const std::string& host_name, int process_id) override;

  void DisposeJoin(const ChangeMsg& msg);
  void DisposeLeave(const ChangeMsg& msg);

  void ScanMessageType(const ChangeMsg& msg);

  ExemptedMessageTypes exempted_msg_types_;

  Graph node_graph_;
  // key: node_id
  WriterWarehouse node_writers_;
  ReaderWarehouse node_readers_;

  // key: channel_id
  WriterWarehouse channel_writers_;
  ReaderWarehouse channel_readers_;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  //  CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_CHANNEL_MANAGER_H_
