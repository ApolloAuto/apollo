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

#include "cyber/service_discovery/specific_manager/service_manager.h"
#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/util.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace service_discovery {

ServiceManager::ServiceManager() {
  allowed_role_ |= 1 << RoleType::ROLE_SERVER;
  allowed_role_ |= 1 << RoleType::ROLE_CLIENT;
  change_type_ = ChangeType::CHANGE_SERVICE;
  channel_name_ = "service_change_broadcast";
}

ServiceManager::~ServiceManager() {}

bool ServiceManager::HasService(const std::string& service_name) {
  uint64_t key = common::Hash(service_name);
  return servers_.Search(key);
}

void ServiceManager::GetServers(RoleAttrVec* servers) {
  RETURN_IF_NULL(servers);
  servers_.GetAllRoles(servers);
}

void ServiceManager::GetClients(const std::string& service_name,
                                RoleAttrVec* clients) {
  RETURN_IF_NULL(clients);
  uint64_t key = common::Hash(service_name);
  clients_.Search(key, clients);
}

bool ServiceManager::Check(const RoleAttributes& attr) {
  RETURN_VAL_IF(!attr.has_service_name(), false);
  RETURN_VAL_IF(!attr.has_service_id(), false);
  return true;
}

void ServiceManager::Dispose(const ChangeMsg& msg) {
  if (msg.operate_type() == OperateType::OPT_JOIN) {
    DisposeJoin(msg);
  } else {
    DisposeLeave(msg);
  }
  Notify(msg);
}

void ServiceManager::OnTopoModuleLeave(const std::string& host_name,
                                       int process_id) {
  RETURN_IF(!is_discovery_started_.load());

  RoleAttributes attr;
  attr.set_host_name(host_name);
  attr.set_process_id(process_id);

  std::vector<RolePtr> servers_to_remove;
  servers_.Search(attr, &servers_to_remove);
  for (auto& server : servers_to_remove) {
    servers_.Remove(server->attributes().service_id());
  }

  std::vector<RolePtr> clients_to_remove;
  clients_.Search(attr, &clients_to_remove);
  for (auto& client : clients_to_remove) {
    clients_.Remove(client->attributes().service_id(), client);
  }

  ChangeMsg msg;
  for (auto& server : servers_to_remove) {
    Convert(server->attributes(), RoleType::ROLE_SERVER, OperateType::OPT_LEAVE,
            &msg);
    Notify(msg);
  }

  for (auto& client : clients_to_remove) {
    Convert(client->attributes(), RoleType::ROLE_CLIENT, OperateType::OPT_LEAVE,
            &msg);
    Notify(msg);
  }
}

void ServiceManager::DisposeJoin(const ChangeMsg& msg) {
  if (msg.role_type() == RoleType::ROLE_SERVER) {
    auto role = std::make_shared<RoleServer>(msg.role_attr());
    servers_.Add(role->attributes().service_id(), role);
  } else {
    auto role = std::make_shared<RoleClient>(msg.role_attr());
    clients_.Add(role->attributes().service_id(), role);
  }
}

void ServiceManager::DisposeLeave(const ChangeMsg& msg) {
  if (msg.role_type() == RoleType::ROLE_SERVER) {
    auto role = std::make_shared<RoleServer>(msg.role_attr());
    servers_.Remove(role->attributes().service_id());
  } else {
    auto role = std::make_shared<RoleClient>(msg.role_attr());
    clients_.Remove(role->attributes().service_id(), role);
  }
}

}  // namespace service_discovery
}  // namespace cyber
}  // namespace apollo
