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

#ifndef CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_NODE_MANAGER_H_
#define CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_NODE_MANAGER_H_

#include <memory>
#include <string>
#include <vector>

#include "cyber/service_discovery/container/single_value_warehouse.h"
#include "cyber/service_discovery/role/role.h"
#include "cyber/service_discovery/specific_manager/manager.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class TopologyManager;

/**
 * @class NodeManager
 * @brief Topology Manager of Node related
 */
class NodeManager : public Manager {
  friend class TopologyManager;

 public:
  using RoleAttrVec = std::vector<RoleAttributes>;
  using NodeWarehouse = SingleValueWarehouse;

  /**
   * @brief Construct a new Node Manager object
   */
  NodeManager();

  /**
   * @brief Destroy the Node Manager object
   */
  virtual ~NodeManager();

  /**
   * @brief Checkout whether we have `node_name` in topology
   *
   * @param node_name Node's name we want to inquire
   * @return true if this node found
   * @return false if this node not exits
   */
  bool HasNode(const std::string& node_name);

  /**
   * @brief Get the Nodes object
   *
   * @param nodes result RoleAttr vector
   */
  void GetNodes(RoleAttrVec* nodes);

 private:
  bool Check(const RoleAttributes& attr) override;
  void Dispose(const ChangeMsg& msg) override;
  void OnTopoModuleLeave(const std::string& host_name, int process_id) override;

  void DisposeJoin(const ChangeMsg& msg);
  void DisposeLeave(const ChangeMsg& msg);

  NodeWarehouse nodes_;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  //  CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_NODE_MANAGER_H_
