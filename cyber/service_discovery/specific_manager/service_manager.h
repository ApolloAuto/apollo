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

#ifndef CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_SERVICE_MANAGER_H_
#define CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_SERVICE_MANAGER_H_

#include <memory>
#include <string>
#include <vector>

#include "cyber/service_discovery/container/multi_value_warehouse.h"
#include "cyber/service_discovery/container/single_value_warehouse.h"
#include "cyber/service_discovery/role/role.h"
#include "cyber/service_discovery/specific_manager/manager.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

class TopologyManager;

/**
 * @class ServiceManager
 * @brief Topology Manager of Service related
 */
class ServiceManager : public Manager {
  friend class TopologyManager;

 public:
  using RoleAttrVec = std::vector<RoleAttributes>;
  using ServerWarehouse = SingleValueWarehouse;
  using ClientWarehouse = MultiValueWarehouse;

  /**
   * @brief Construct a new Service Manager object
   */
  ServiceManager();

  /**
   * @brief Destroy the Service Manager object
   */
  virtual ~ServiceManager();

  /**
   * @brief Inquire whether `service_name` exists in topology
   *
   * @param service_name the name we inquire
   * @return true if service exists
   * @return false if service not exists
   */
  bool HasService(const std::string& service_name);

  /**
   * @brief Get the All Server in the topology
   *
   * @param servers result RoleAttr vector
   */
  void GetServers(RoleAttrVec* servers);

  /**
   * @brief Get the Clients object that subscribes `service_name`
   *
   * @param service_name Name of service you want to get
   * @param clients result vector
   */
  void GetClients(const std::string& service_name, RoleAttrVec* clients);

 private:
  bool Check(const RoleAttributes& attr) override;
  void Dispose(const ChangeMsg& msg) override;
  void OnTopoModuleLeave(const std::string& host_name, int process_id) override;

  void DisposeJoin(const ChangeMsg& msg);
  void DisposeLeave(const ChangeMsg& msg);

  ServerWarehouse servers_;
  ClientWarehouse clients_;
};

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo

#endif  //  CYBER_SERVICE_DISCOVERY_SPECIFIC_MANAGER_SERVICE_MANAGER_H_
