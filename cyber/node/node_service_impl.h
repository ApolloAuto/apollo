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

#ifndef CYBER_NODE_NODE_SERVICE_IMPL_H_
#define CYBER_NODE_NODE_SERVICE_IMPL_H_

#include <memory>
#include <string>
#include <vector>

#include "cyber/common/global_data.h"
#include "cyber/service/client.h"
#include "cyber/service/service.h"
#include "cyber/service_discovery/topology_manager.h"

namespace apollo {
namespace cyber {

class Node;

/**
 * @class NodeServiceImpl
 * @brief The implementation for Node to create Objects connected by Param.
 * e.g. Param Server and Client
 */
class NodeServiceImpl {
 public:
  friend class Node;

  /**
   * @brief Construct a new Node Service Impl object
   *
   * @param node_name node name
   */
  explicit NodeServiceImpl(const std::string& node_name)
      : node_name_(node_name) {
    attr_.set_host_name(common::GlobalData::Instance()->HostName());
    attr_.set_process_id(common::GlobalData::Instance()->ProcessId());
    attr_.set_node_name(node_name);
    auto node_id = common::GlobalData::RegisterNode(node_name);
    attr_.set_node_id(node_id);
  }

  /**
   * @brief Forbid default-constructor
   */
  NodeServiceImpl() = delete;

  /**
   * @brief Destroy the Node Service Impl object
   *
   */
  ~NodeServiceImpl() {}

 private:
  template <typename Request, typename Response>
  auto CreateService(const std::string& service_name,
                     const typename Service<Request, Response>::ServiceCallback&
                         service_callback) ->
      typename std::shared_ptr<Service<Request, Response>>;

  template <typename Request, typename Response>
  auto CreateClient(const std::string& service_name) ->
      typename std::shared_ptr<Client<Request, Response>>;

  std::vector<std::weak_ptr<ServiceBase>> service_list_;
  std::vector<std::weak_ptr<ClientBase>> client_list_;
  std::string node_name_;
  proto::RoleAttributes attr_;
};

template <typename Request, typename Response>
auto NodeServiceImpl::CreateService(
    const std::string& service_name,
    const typename Service<Request, Response>::ServiceCallback&
        service_callback) ->
    typename std::shared_ptr<Service<Request, Response>> {
  auto service_ptr = std::make_shared<Service<Request, Response>>(
      node_name_, service_name, service_callback);
  RETURN_VAL_IF(!service_ptr->Init(), nullptr);

  service_list_.emplace_back(service_ptr);
  attr_.set_service_name(service_name);
  auto service_id = common::GlobalData::RegisterService(service_name);
  attr_.set_service_id(service_id);
  service_discovery::TopologyManager::Instance()->service_manager()->Join(
      attr_, RoleType::ROLE_SERVER);
  return service_ptr;
}

template <typename Request, typename Response>
auto NodeServiceImpl::CreateClient(const std::string& service_name) ->
    typename std::shared_ptr<Client<Request, Response>> {
  auto client_ptr =
      std::make_shared<Client<Request, Response>>(node_name_, service_name);
  RETURN_VAL_IF(!client_ptr->Init(), nullptr);

  client_list_.emplace_back(client_ptr);
  attr_.set_service_name(service_name);
  auto service_id = common::GlobalData::RegisterService(service_name);
  attr_.set_service_id(service_id);
  service_discovery::TopologyManager::Instance()->service_manager()->Join(
      attr_, RoleType::ROLE_CLIENT);
  return client_ptr;
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_NODE_NODE_SERVICE_IMPL_H_
