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

#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "cybertron/common/global_data.h"
#include "cybertron/cybertron.h"
#include "cybertron/time/time.h"
#include "cybertron/service_discovery/topology_manager.h"
#include "cybertron/transport/common/identity.h"

using apollo::cybertron::Time;
using apollo::cybertron::common::GlobalData;
using apollo::cybertron::proto::ChangeMsg;
using apollo::cybertron::proto::ChangeType;
using apollo::cybertron::proto::OperateType;
using apollo::cybertron::proto::RoleAttributes;
using apollo::cybertron::proto::RoleType;
using apollo::cybertron::service_discovery::RoleBase;
using apollo::cybertron::service_discovery::RolePtr;
using apollo::cybertron::service_discovery::TopologyManager;
using apollo::cybertron::transport::Identity;

using AttrContainer = std::vector<RoleAttributes>;

const uint64_t LARGE_TIME = 123456789;

enum TopologyRole {
  TOPO_NODE = 1,
  TOPO_WRITER,
  TOPO_READER,
  TOPO_SERVICE,
  TOPO_CLIENT = 5,
  TOPO_ROLE_MAX,
};

struct RunConfig {
  RunConfig() {
    for (int i = TOPO_NODE; i < TOPO_ROLE_MAX; ++i) {
      role_num[i] = 100;
    }
  }

  void Display() {
    AINFO << "********** Run Config **********";
    AINFO << "   node number: " << role_num[TOPO_NODE];
    AINFO << " writer number: " << role_num[TOPO_WRITER];
    AINFO << " reader number: " << role_num[TOPO_READER];
    AINFO << "service number: " << role_num[TOPO_SERVICE];
    AINFO << " client number: " << role_num[TOPO_CLIENT];
    AINFO << "********************************";
  }

  std::map<int, uint32_t> role_num;
};

void Usage() {
  AINFO << "Usage:(all number must less than 256)";
  AINFO << "    argv[0] program name";
  AINFO << "    argv[1] node number";
  AINFO << "    argv[2] writer number";
  AINFO << "    argv[3] reader number";
  AINFO << "    argv[4] service number";
  AINFO << "    argv[5] client number";
}

bool Parse(int argc, char* argv[], RunConfig* cfg) {
  if (cfg == nullptr) {
    return false;
  }

  for (int i = 1; i < argc; ++i) {
    int tmp = 0;
    tmp = atoi(argv[i]);
    if (tmp <= 0 || tmp > 255) {
      AINFO << "argv[" << i << "] invalid";
      return false;
    }
    cfg->role_num[i] = tmp;
  }

  return true;
}

void FillInAttr(const ChangeType& change_type, AttrContainer* container) {
  if (container == nullptr) {
    return;
  }

  uint32_t count = 0;
  for (auto itr = container->begin(); itr != container->end(); ++itr) {
    itr->set_host_name(GlobalData::Instance()->HostName());
    itr->set_process_id(GlobalData::Instance()->ProcessId());
    std::string node_name = "node_" + std::to_string(count) + "_" +
                            std::to_string(itr->process_id());
    itr->set_node_name(node_name);
    uint64_t node_id = GlobalData::RegisterNode(node_name);
    itr->set_node_id(node_id);
    if (change_type == ChangeType::CHANGE_CHANNEL) {
      std::string channel_name = "channel_" + std::to_string(count) + "_" +
                                 std::to_string(itr->process_id());
      itr->set_channel_name(channel_name);
      uint64_t channel_id = GlobalData::RegisterChannel(channel_name);
      itr->set_channel_id(channel_id);
      Identity id;
      itr->set_id(id.HashValue());
    }
    if (change_type == ChangeType::CHANGE_SERVICE) {
      std::string service_name = "service_" + std::to_string(count) + "_" +
                                 std::to_string(itr->process_id());
      itr->set_service_name(service_name);
      uint64_t service_id = GlobalData::RegisterService(service_name);
      itr->set_service_id(service_id);
    }
    ++count;
  }
}

void Run(RunConfig& cfg) {
  auto node_mgr = TopologyManager::Instance()->node_manager();
  auto channel_mgr = TopologyManager::Instance()->channel_manager();
  auto service_mgr = TopologyManager::Instance()->service_manager();

  AttrContainer nodes(cfg.role_num[TOPO_NODE]);
  FillInAttr(ChangeType::CHANGE_NODE, &nodes);

  AttrContainer writers(cfg.role_num[TOPO_WRITER]);
  FillInAttr(ChangeType::CHANGE_CHANNEL, &writers);

  AttrContainer readers(cfg.role_num[TOPO_READER]);
  FillInAttr(ChangeType::CHANGE_CHANNEL, &readers);

  AttrContainer services(cfg.role_num[TOPO_SERVICE]);
  FillInAttr(ChangeType::CHANGE_SERVICE, &services);

  AttrContainer clients(cfg.role_num[TOPO_CLIENT]);
  FillInAttr(ChangeType::CHANGE_SERVICE, &clients);

  for (auto& item : nodes) {
    node_mgr->Join(item, RoleType::ROLE_NODE);
  }
  for (auto& item : writers) {
    channel_mgr->Join(item, RoleType::ROLE_WRITER);
  }
  for (auto& item : readers) {
    channel_mgr->Join(item, RoleType::ROLE_READER);
  }
  for (auto& item : services) {
    service_mgr->Join(item, RoleType::ROLE_SERVER);
  }
  for (auto& item : clients) {
    service_mgr->Join(item, RoleType::ROLE_CLIENT);
  }
}

int main(int argc, char* argv[]) {
  if (argc > 6) {
    Usage();
    return -1;
  }
  apollo::cybertron::Init(argv[0]);

  RunConfig cfg;
  Parse(argc, argv, &cfg);

  Run(cfg);

  AINFO << "Run over, wait for shutdown.";

  apollo::cybertron::WaitForShutdown();

  return 0;
}
