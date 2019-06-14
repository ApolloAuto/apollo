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

#ifndef CYBER_SERVICE_DISCOVERY_TOPOLOGY_H_
#define CYBER_SERVICE_DISCOVERY_TOPOLOGY_H_

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "cyber/base/signal.h"
#include "cyber/common/macros.h"
#include "cyber/service_discovery/communication/participant_listener.h"
#include "cyber/service_discovery/specific_manager/channel_manager.h"
#include "cyber/service_discovery/specific_manager/node_manager.h"
#include "cyber/service_discovery/specific_manager/service_manager.h"
#include "cyber/transport/rtps/participant.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class NodeManager;
using NodeManagerPtr = std::shared_ptr<NodeManager>;

class ChannelManager;
using ChannelManagerPtr = std::shared_ptr<ChannelManager>;

class ServiceManager;
using ServiceManagerPtr = std::shared_ptr<ServiceManager>;

/**
 * @class TopologyManager
 * @brief elements in Cyber -- Node, Channel, Service, Writer, Reader, Client
 * and Server's relationship is presented by Topology. You can Imagine that a
 * directed graph -- Node is the container of Server/Client/Writer/Reader, and
 * they are the vertice of the graph and Channel is the Edge from Writer flow to
 * the Reader, Service is the Edge from Server to Client. Thus we call Writer
 * and Server `Upstream`, Reader and Client `Downstream` To generate this graph,
 * we use TopologyManager, it has three sub managers -- NodeManager: You can
 * find Nodes in this topology ChannelManager: You can find Channels in this
 * topology, and their Writers and Readers ServiceManager: You can find Services
 * in this topology, and their Servers and Clients TopologyManager use
 * fast-rtps' Participant to communicate. It can broadcast Join or Leave
 * messages of those elements. Also, you can register you own `ChangeFunc` to
 * monitor topology change
 */
class TopologyManager {
 public:
  using ChangeSignal = base::Signal<const ChangeMsg&>;
  using ChangeFunc = std::function<void(const ChangeMsg&)>;
  using ChangeConnection = base::Connection<const ChangeMsg&>;
  using PartNameContainer =
      std::map<eprosima::fastrtps::rtps::GUID_t, std::string>;
  using PartInfo = eprosima::fastrtps::ParticipantDiscoveryInfo;

  virtual ~TopologyManager();

  /**
   * @brief Shutdown the TopologyManager
   */
  void Shutdown();

  /**
   * @brief To observe the topology change, you can register a `ChangeFunc`
   *
   * @param func is the observe function
   * @return ChangeConnection is the connection that connected to
   * `change_signal_`. Used to Remove your observe function
   */
  ChangeConnection AddChangeListener(const ChangeFunc& func);

  /**
   * @brief Remove the observe function connect to `change_signal_` by `conn`
   */
  void RemoveChangeListener(const ChangeConnection& conn);

  /**
   * @brief Get shared_ptr for NodeManager
   */
  NodeManagerPtr& node_manager() { return node_manager_; }

  /**
   * @brief Get shared_ptr for ChannelManager
   */
  ChannelManagerPtr& channel_manager() { return channel_manager_; }

  /**
   * @brief Get shared_ptr for ServiceManager
   */
  ServiceManagerPtr& service_manager() { return service_manager_; }

 private:
  bool Init();

  bool InitNodeManager();
  bool InitChannelManager();
  bool InitServiceManager();

  bool CreateParticipant();
  void OnParticipantChange(const PartInfo& info);
  bool Convert(const PartInfo& info, ChangeMsg* change_msg);
  bool ParseParticipantName(const std::string& participant_name,
                            std::string* host_name, int* process_id);

  std::atomic<bool> init_;             /// Is TopologyManager inited
  NodeManagerPtr node_manager_;        /// shared ptr of NodeManager
  ChannelManagerPtr channel_manager_;  /// shared ptr of ChannelManager
  ServiceManagerPtr service_manager_;  /// shared ptr of ServiceManager
  /// rtps participant to publish and subscribe
  transport::ParticipantPtr participant_;
  ParticipantListener* participant_listener_;
  ChangeSignal change_signal_;           /// topology changing signal,
                                         ///< connect to `ChangeFunc`s
  PartNameContainer participant_names_;  /// other participant in the topology

  DECLARE_SINGLETON(TopologyManager)
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SERVICE_DISCOVERY_TOPOLOGY_H_
