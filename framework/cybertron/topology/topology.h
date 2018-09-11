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

#ifndef CYBERTRON_TOPOLOGY_TOPOLOGY_H_
#define CYBERTRON_TOPOLOGY_TOPOLOGY_H_

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <fastrtps/participant/Participant.h>

#include "cybertron/base/signal.h"
#include "cybertron/common/macros.h"
#include "cybertron/topology/communication/participant_listener.h"
#include "cybertron/topology/manager/channel_manager.h"
#include "cybertron/topology/manager/node_manager.h"
#include "cybertron/topology/manager/service_manager.h"
#include "cybertron/transport/rtps/participant.h"

namespace apollo {
namespace cybertron {
namespace topology {

class NodeManager;
using NodeManagerPtr = std::shared_ptr<NodeManager>;

class ChannelManager;
using ChannelManagerPtr = std::shared_ptr<ChannelManager>;

class ServiceManager;
using ServiceManagerPtr = std::shared_ptr<ServiceManager>;

class Topology {
 public:
  using ChangeSignal = base::Signal<const ChangeMsg&>;
  using ChangeFunc = std::function<void(const ChangeMsg&)>;
  using ChangeConnection = base::Connection<const ChangeMsg&>;
  using PartNameContainer =
      std::map<eprosima::fastrtps::rtps::GUID_t, std::string>;
  using PartInfo = eprosima::fastrtps::ParticipantDiscoveryInfo;

  virtual ~Topology();

  void Shutdown();

  ChangeConnection AddChangeListener(const ChangeFunc& func);
  void RemoveChangeListener(const ChangeConnection& conn);

  NodeManagerPtr& node_manager() { return node_manager_; }
  ChannelManagerPtr& channel_manager() { return channel_manager_; }
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

  std::atomic<bool> init_;
  NodeManagerPtr node_manager_;
  ChannelManagerPtr channel_manager_;
  ServiceManagerPtr service_manager_;
  transport::ParticipantPtr participant_;
  ParticipantListener* participant_listener_;
  ChangeSignal change_signal_;
  PartNameContainer participant_names_;

  DECLARE_SINGLETON(Topology)
};

}  // namespace topology
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TOPOLOGY_TOPOLOGY_H_
