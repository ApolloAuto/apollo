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

/**
 * @class ChannelManager
 * @brief Topology Manager of Service related
 */
class ChannelManager : public Manager {
  friend class TopologyManager;

 public:
  using RoleAttrVec = std::vector<proto::RoleAttributes>;
  using WriterWarehouse = MultiValueWarehouse;
  using ReaderWarehouse = MultiValueWarehouse;
  using ExemptedMessageTypes = std::unordered_set<std::string>;

  /**
   * @brief Construct a new Channel Manager object
   */
  ChannelManager();

  /**
   * @brief Destroy the Channel Manager object
   */
  virtual ~ChannelManager();

  /**
   * @brief Get all channel names in the topology
   *
   * @param channels result vector
   */
  void GetChannelNames(std::vector<std::string>* channels);

  /**
   * @brief Get the Protocol Desc of `channel_name`
   *
   * @param channel_name channel name we want to inquire
   * @param proto_desc result string, empty if inquire failed
   */
  void GetProtoDesc(const std::string& channel_name, std::string* proto_desc);

  /**
   * @brief Get the Msg Type of `channel_name`
   *
   * @param channel_name channel name we want to inquire
   * @param msg_type result string, empty if inquire failed
   */
  void GetMsgType(const std::string& channel_name, std::string* msg_type);

  /**
   * @brief Inquire if there is at least one Writer that publishes
   * `channel_name`
   *
   * @param channel_name channel name we want to inquire
   * @return true if there is at least one Writer
   * @return false if there are no Writers
   */
  bool HasWriter(const std::string& channel_name);

  /**
   * @brief Get All Writers object
   *
   * @param writers result RoleAttr vector
   */
  void GetWriters(RoleAttrVec* writers);

  /**
   * @brief Get the Writers Of Node object
   *
   * @param node_name node's name we want to inquire
   * @param writers result RoleAttribute vector
   */
  void GetWritersOfNode(const std::string& node_name, RoleAttrVec* writers);

  /**
   * @brief Get the Writers Of Channel object
   *
   * @param channel_name channel's name we want to inquire
   * @param writers result RoleAttribute vector
   */
  void GetWritersOfChannel(const std::string& channel_name,
                           RoleAttrVec* writers);

  /**
   * @brief Inquire if there is at least one Reader that publishes
   * `channel_name`
   *
   * @param channel_name channel name we want to inquire
   * @return true if there is at least one Reader
   * @return false if there are no Reader
   */
  bool HasReader(const std::string& channel_name);

  /**
   * @brief Get All Readers object
   *
   * @param readers result RoleAttr vector
   */
  void GetReaders(RoleAttrVec* readers);

  /**
   * @brief Get the Readers Of Node object
   *
   * @param node_name node's name we want to inquire
   * @param readers result RoleAttribute vector
   */
  void GetReadersOfNode(const std::string& node_name, RoleAttrVec* readers);

  /**
   * @brief Get the Readers Of Channel object
   *
   * @param channel_name channel's name we want to inquire
   * @param readers result RoleAttribute vector
   */
  void GetReadersOfChannel(const std::string& channel_name,
                           RoleAttrVec* readers);

  /**
   * @brief Get the Upstream Of Node object.
   * If Node A has writer that publishes channel-1, and Node B has reader that
   * subscribes channel-1 then A is B's Upstream node, and B is A's Downstream
   * node
   *
   * @param node_name node's name we want to inquire
   * @param upstream_nodes result RoleAttribute vector
   */
  void GetUpstreamOfNode(const std::string& node_name,
                         RoleAttrVec* upstream_nodes);

  /**
   * @brief Get the Downstream Of Node object.
   * If Node A has writer that publishes channel-1, and Node B has reader that
   * subscribes channel-1 then A is B's Upstream node, and B is A's Downstream
   * node
   *
   * @param node_name node's name we want to inquire
   * @param downstream_nodes result RoleAttribute vector
   */
  void GetDownstreamOfNode(const std::string& node_name,
                           RoleAttrVec* downstream_nodes);

  /**
   * @brief Get the Flow Direction from `lhs_node_node` to `rhs_node_name`
   * You can see FlowDirection's description for more information
   * @return FlowDirection result direction
   */
  FlowDirection GetFlowDirection(const std::string& lhs_node_name,
                                 const std::string& rhs_node_name);

  /**
   * @brief Is `lhs` and `rhs` have same MessageType
   *
   * @param lhs the left message type to compare
   * @param rhs the right message type to compare
   * @return true if type matches
   * @return false if type does not matches
   */
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
