/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "cybertron/time/time.h"
#include "cybertron/topology/topology.h"
#include "cybertron/transport/common/identity.h"

using apollo::cybertron::Time;
using apollo::cybertron::common::GlobalData;
using apollo::cybertron::proto::ChangeMsg;
using apollo::cybertron::proto::ChangeType;
using apollo::cybertron::proto::OperateType;
using apollo::cybertron::proto::RoleAttributes;
using apollo::cybertron::proto::RoleType;
using apollo::cybertron::topology::RoleBase;
using apollo::cybertron::topology::RolePtr;
using apollo::cybertron::topology::Topology;
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
    std::cout << "********** Run Config **********" << std::endl;
    std::cout << "   node number: " << role_num[TOPO_NODE] << std::endl;
    std::cout << " writer number: " << role_num[TOPO_WRITER] << std::endl;
    std::cout << " reader number: " << role_num[TOPO_READER] << std::endl;
    std::cout << "service number: " << role_num[TOPO_SERVICE] << std::endl;
    std::cout << " client number: " << role_num[TOPO_CLIENT] << std::endl;
    std::cout << "********************************" << std::endl;
  }

  std::map<int, uint32_t> role_num;
};

struct RunStatistics {
  RunStatistics()
      : min_join_single_role_time(LARGE_TIME),
        max_join_single_role_time(0),
        avg_join_single_role_time(0),
        total_join_roles_time(0),
        min_get_single_role_time(LARGE_TIME),
        max_get_single_role_time(0),
        avg_get_single_role_time(0),
        total_get_roles_time(0),
        get_total_roles_time(0),
        min_leave_single_role_time(LARGE_TIME),
        max_leave_single_role_time(0),
        avg_leave_single_role_time(0),
        total_leave_roles_time(0),
        error_counts(0) {}

  void Display() {
    std::cout << "\nJoin Performance(time unit: ns)" << std::endl;
    std::cout << "min_join_single_role_time: " << min_join_single_role_time
              << std::endl;
    std::cout << "max_join_single_role_time: " << max_join_single_role_time
              << std::endl;
    std::cout << "avg_join_single_role_time: " << avg_join_single_role_time
              << std::endl;
    std::cout << "    total_join_roles_time: " << total_join_roles_time
              << std::endl;

    std::cout << "\nGet Performance(time unit: ns)" << std::endl;
    std::cout << "min_get_single_role_time: " << min_get_single_role_time
              << std::endl;
    std::cout << "max_get_single_role_time: " << max_get_single_role_time
              << std::endl;
    std::cout << "avg_get_single_role_time: " << avg_get_single_role_time
              << std::endl;
    std::cout << "    total_get_roles_time: " << total_get_roles_time
              << std::endl;
    std::cout << "    get_total_roles_time: " << get_total_roles_time
              << std::endl;

    std::cout << "\nLeave Performance(time unit: ns)" << std::endl;
    std::cout << "min_leave_single_role_time: " << min_leave_single_role_time
              << std::endl;
    std::cout << "max_leave_single_role_time: " << max_leave_single_role_time
              << std::endl;
    std::cout << "avg_leave_single_role_time: " << avg_leave_single_role_time
              << std::endl;
    std::cout << "    total_leave_roles_time: " << total_leave_roles_time
              << std::endl;

    std::cout << "\nErrors" << std::endl;
    std::cout << "error counts: " << error_counts << std::endl;
  }

  uint64_t min_join_single_role_time;
  uint64_t max_join_single_role_time;
  uint64_t avg_join_single_role_time;
  uint64_t total_join_roles_time;

  uint64_t min_get_single_role_time;
  uint64_t max_get_single_role_time;
  uint64_t avg_get_single_role_time;
  uint64_t total_get_roles_time;
  uint64_t get_total_roles_time;

  uint64_t min_leave_single_role_time;
  uint64_t max_leave_single_role_time;
  uint64_t avg_leave_single_role_time;
  uint64_t total_leave_roles_time;

  uint64_t error_counts;
};

void Usage() {
  std::cout << "Usage:(all number must less than 256)" << std::endl;
  std::cout << "    argv[0] program name" << std::endl;
  std::cout << "    argv[1] node number" << std::endl;
  std::cout << "    argv[2] writer number" << std::endl;
  std::cout << "    argv[3] reader number" << std::endl;
  std::cout << "    argv[4] service number" << std::endl;
  std::cout << "    argv[5] client number" << std::endl;
}

bool Parse(int argc, char* argv[], RunConfig* cfg) {
  if (cfg == nullptr) {
    return false;
  }

  for (int i = 1; i < argc; ++i) {
    int tmp = 0;
    tmp = atoi(argv[i]);
    if (tmp <= 0 || tmp > 255) {
      std::cout << "argv[" << i << "] invalid" << std::endl;
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
      std::string channel_name = "channel_" + std::to_string(count);
      itr->set_channel_name(channel_name);
      uint64_t channel_id = GlobalData::RegisterChannel(channel_name);
      itr->set_channel_id(channel_id);
      Identity id;
      itr->set_id(id.HashValue());
    }
    if (change_type == ChangeType::CHANGE_SERVICE) {
      std::string service_name = "service_" + std::to_string(count);
      itr->set_service_name(service_name);
      uint64_t service_id = GlobalData::RegisterService(service_name);
      itr->set_service_id(service_id);
    }
    ++count;
  }
}

void RunNodeTest(const uint32_t& num) {
  if (num == 0) {
    return;
  }
  auto node_manager = Topology::Instance()->node_manager();
  AttrContainer nodes(num);
  FillInAttr(ChangeType::CHANGE_NODE, &nodes);

  RunStatistics node_stat;

  // Join
  uint64_t start_time = Time::Now().ToNanosecond();
  for (auto& item : nodes) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    node_manager->Join(item, RoleType::ROLE_NODE);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < node_stat.min_join_single_role_time) {
      node_stat.min_join_single_role_time = this_cost_time;
    }
    if (this_cost_time > node_stat.max_join_single_role_time) {
      node_stat.max_join_single_role_time = this_cost_time;
    }
  }
  uint64_t end_time = Time::Now().ToNanosecond();
  node_stat.total_join_roles_time = end_time - start_time;
  node_stat.avg_join_single_role_time = node_stat.total_join_roles_time / num;

  // Get
  for (auto& item : nodes) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    if (!node_manager->HasNode(item.node_name())) {
      ++node_stat.error_counts;
    }
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < node_stat.min_get_single_role_time) {
      node_stat.min_get_single_role_time = this_cost_time;
    }
    if (this_cost_time > node_stat.max_get_single_role_time) {
      node_stat.max_get_single_role_time = this_cost_time;
    }
    node_stat.total_get_roles_time += this_cost_time;
  }
  node_stat.avg_get_single_role_time = node_stat.total_get_roles_time / num;

  std::vector<RoleAttributes> search_nodes;
  start_time = Time::Now().ToNanosecond();
  node_manager->GetNodes(&search_nodes);
  end_time = Time::Now().ToNanosecond();
  node_stat.get_total_roles_time = end_time - start_time;

  if (search_nodes.size() != num) {
    ++node_stat.error_counts;
  }

  // Leave
  start_time = Time::Now().ToNanosecond();
  for (auto& item : nodes) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    node_manager->Leave(item, RoleType::ROLE_NODE);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < node_stat.min_leave_single_role_time) {
      node_stat.min_leave_single_role_time = this_cost_time;
    }
    if (this_cost_time > node_stat.max_leave_single_role_time) {
      node_stat.max_leave_single_role_time = this_cost_time;
    }
  }
  end_time = Time::Now().ToNanosecond();
  node_stat.total_leave_roles_time = end_time - start_time;
  node_stat.avg_leave_single_role_time = node_stat.total_leave_roles_time / num;

  // Show results
  std::cout << "********** NodeManager Run Statistics **********" << std::endl;
  node_stat.Display();
  std::cout << "************************************************" << std::endl;
}

void RunChannelTest(const uint32_t& writer_num, const uint32_t& reader_num) {
  if (writer_num == 0 || reader_num == 0) {
    return;
  }
  auto channel_manager = Topology::Instance()->channel_manager();
  AttrContainer writers(writer_num);
  FillInAttr(ChangeType::CHANGE_CHANNEL, &writers);
  RunStatistics writer_stat;

  AttrContainer readers(reader_num);
  FillInAttr(ChangeType::CHANGE_CHANNEL, &readers);
  RunStatistics reader_stat;

  // Join
  uint64_t start_time = Time::Now().ToNanosecond();
  for (auto& item : writers) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    channel_manager->Join(item, RoleType::ROLE_WRITER);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < writer_stat.min_join_single_role_time) {
      writer_stat.min_join_single_role_time = this_cost_time;
    }
    if (this_cost_time > writer_stat.max_join_single_role_time) {
      writer_stat.max_join_single_role_time = this_cost_time;
    }
  }
  uint64_t end_time = Time::Now().ToNanosecond();
  writer_stat.total_join_roles_time = end_time - start_time;
  writer_stat.avg_join_single_role_time =
      writer_stat.total_join_roles_time / writer_num;

  start_time = Time::Now().ToNanosecond();
  for (auto& item : readers) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    channel_manager->Join(item, RoleType::ROLE_READER);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < reader_stat.min_join_single_role_time) {
      reader_stat.min_join_single_role_time = this_cost_time;
    }
    if (this_cost_time > reader_stat.max_join_single_role_time) {
      reader_stat.max_join_single_role_time = this_cost_time;
    }
  }
  end_time = Time::Now().ToNanosecond();
  reader_stat.total_join_roles_time = end_time - start_time;
  reader_stat.avg_join_single_role_time =
      reader_stat.total_join_roles_time / reader_num;

  // Get
  // writer
  for (auto& item : writers) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    if (!channel_manager->HasWriter(item.channel_name())) {
      ++writer_stat.error_counts;
    }
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < writer_stat.min_get_single_role_time) {
      writer_stat.min_get_single_role_time = this_cost_time;
    }
    if (this_cost_time > writer_stat.max_get_single_role_time) {
      writer_stat.max_get_single_role_time = this_cost_time;
    }
    writer_stat.total_get_roles_time += this_cost_time;
  }
  writer_stat.avg_get_single_role_time =
      writer_stat.total_get_roles_time / writer_num;

  std::vector<RoleAttributes> search_writers;
  start_time = Time::Now().ToNanosecond();
  channel_manager->GetWriters(&search_writers);
  end_time = Time::Now().ToNanosecond();
  writer_stat.get_total_roles_time = end_time - start_time;

  if (search_writers.size() != writer_num) {
    ++writer_stat.error_counts;
  }

  // reader
  for (auto& item : readers) {
    std::vector<RoleAttributes> search_readers;
    uint64_t this_start_time = Time::Now().ToNanosecond();
    channel_manager->GetReadersOfChannel(item.channel_name(), &search_readers);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    if (search_readers.empty()) {
      ++reader_stat.error_counts;
    }
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < reader_stat.min_get_single_role_time) {
      reader_stat.min_get_single_role_time = this_cost_time;
    }
    if (this_cost_time > reader_stat.max_get_single_role_time) {
      reader_stat.max_get_single_role_time = this_cost_time;
    }
    reader_stat.total_get_roles_time += this_cost_time;
  }
  reader_stat.avg_get_single_role_time =
      reader_stat.total_get_roles_time / reader_num;

  std::vector<RoleAttributes> search_readers;
  start_time = Time::Now().ToNanosecond();
  channel_manager->GetReaders(&search_readers);
  end_time = Time::Now().ToNanosecond();
  reader_stat.get_total_roles_time = end_time - start_time;

  if (search_readers.size() != reader_num) {
    ++reader_stat.error_counts;
  }

  // Leave
  // writer
  start_time = Time::Now().ToNanosecond();
  for (auto& item : writers) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    channel_manager->Leave(item, RoleType::ROLE_WRITER);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < writer_stat.min_leave_single_role_time) {
      writer_stat.min_leave_single_role_time = this_cost_time;
    }
    if (this_cost_time > writer_stat.max_leave_single_role_time) {
      writer_stat.max_leave_single_role_time = this_cost_time;
    }
  }
  end_time = Time::Now().ToNanosecond();
  writer_stat.total_leave_roles_time = end_time - start_time;
  writer_stat.avg_leave_single_role_time =
      writer_stat.total_leave_roles_time / writer_num;

  // reader
  start_time = Time::Now().ToNanosecond();
  for (auto& item : readers) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    channel_manager->Leave(item, RoleType::ROLE_READER);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < reader_stat.min_leave_single_role_time) {
      reader_stat.min_leave_single_role_time = this_cost_time;
    }
    if (this_cost_time > reader_stat.max_leave_single_role_time) {
      reader_stat.max_leave_single_role_time = this_cost_time;
    }
  }
  end_time = Time::Now().ToNanosecond();
  reader_stat.total_leave_roles_time = end_time - start_time;
  reader_stat.avg_leave_single_role_time =
      reader_stat.total_leave_roles_time / reader_num;

  // Show results
  std::cout << "********** ChannelManager Run Statistics **********"
            << std::endl;
  std::cout << "***** Writer *****" << std::endl;
  writer_stat.Display();
  std::cout << "******************" << std::endl;
  std::cout << "***** Reader *****" << std::endl;
  reader_stat.Display();
  std::cout << "******************" << std::endl;
  std::cout << "************************************************" << std::endl;
}

void RunServiceTest(const uint32_t& service_num, const uint32_t& client_num) {
  if (service_num == 0 || client_num == 0) {
    return;
  }
  auto service_manager = Topology::Instance()->service_manager();
  AttrContainer services(service_num);
  FillInAttr(ChangeType::CHANGE_SERVICE, &services);
  RunStatistics service_stat;

  AttrContainer clients(client_num);
  FillInAttr(ChangeType::CHANGE_SERVICE, &clients);
  RunStatistics client_stat;

  // Join
  uint64_t start_time = Time::Now().ToNanosecond();
  for (auto& item : services) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    service_manager->Join(item, RoleType::ROLE_SERVER);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < service_stat.min_join_single_role_time) {
      service_stat.min_join_single_role_time = this_cost_time;
    }
    if (this_cost_time > service_stat.max_join_single_role_time) {
      service_stat.max_join_single_role_time = this_cost_time;
    }
  }
  uint64_t end_time = Time::Now().ToNanosecond();
  service_stat.total_join_roles_time = end_time - start_time;
  service_stat.avg_join_single_role_time =
      service_stat.total_join_roles_time / service_num;

  start_time = Time::Now().ToNanosecond();
  for (auto& item : clients) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    service_manager->Join(item, RoleType::ROLE_CLIENT);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < client_stat.min_join_single_role_time) {
      client_stat.min_join_single_role_time = this_cost_time;
    }
    if (this_cost_time > client_stat.max_join_single_role_time) {
      client_stat.max_join_single_role_time = this_cost_time;
    }
  }
  end_time = Time::Now().ToNanosecond();
  client_stat.total_join_roles_time = end_time - start_time;
  client_stat.avg_join_single_role_time =
      client_stat.total_join_roles_time / client_num;

  // Get
  // service
  for (auto& item : services) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    if (!service_manager->HasService(item.service_name())) {
      ++service_stat.error_counts;
    }
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < service_stat.min_get_single_role_time) {
      service_stat.min_get_single_role_time = this_cost_time;
    }
    if (this_cost_time > service_stat.max_get_single_role_time) {
      service_stat.max_get_single_role_time = this_cost_time;
    }
    service_stat.total_get_roles_time += this_cost_time;
  }
  service_stat.avg_get_single_role_time =
      service_stat.total_get_roles_time / service_num;

  std::vector<RoleAttributes> search_services;
  start_time = Time::Now().ToNanosecond();
  service_manager->GetServers(&search_services);
  end_time = Time::Now().ToNanosecond();
  service_stat.get_total_roles_time = end_time - start_time;

  if (search_services.size() != service_num) {
    ++service_stat.error_counts;
  }

  // client
  for (auto& item : clients) {
    std::vector<RoleAttributes> search_clients;
    uint64_t this_start_time = Time::Now().ToNanosecond();
    service_manager->GetClients(item.service_name(), &search_clients);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    if (search_clients.empty()) {
      ++client_stat.error_counts;
    }
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < client_stat.min_get_single_role_time) {
      client_stat.min_get_single_role_time = this_cost_time;
    }
    if (this_cost_time > client_stat.max_get_single_role_time) {
      client_stat.max_get_single_role_time = this_cost_time;
    }
    client_stat.total_get_roles_time += this_cost_time;
  }
  client_stat.avg_get_single_role_time =
      client_stat.total_get_roles_time / client_num;

  // Leave
  // service
  start_time = Time::Now().ToNanosecond();
  for (auto& item : services) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    service_manager->Leave(item, RoleType::ROLE_SERVER);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < service_stat.min_leave_single_role_time) {
      service_stat.min_leave_single_role_time = this_cost_time;
    }
    if (this_cost_time > service_stat.max_leave_single_role_time) {
      service_stat.max_leave_single_role_time = this_cost_time;
    }
  }
  end_time = Time::Now().ToNanosecond();
  service_stat.total_leave_roles_time = end_time - start_time;
  service_stat.avg_leave_single_role_time =
      service_stat.total_leave_roles_time / service_num;

  // client
  start_time = Time::Now().ToNanosecond();
  for (auto& item : clients) {
    uint64_t this_start_time = Time::Now().ToNanosecond();
    service_manager->Leave(item, RoleType::ROLE_CLIENT);
    uint64_t this_end_time = Time::Now().ToNanosecond();
    uint64_t this_cost_time = this_end_time - this_start_time;
    if (this_cost_time < client_stat.min_leave_single_role_time) {
      client_stat.min_leave_single_role_time = this_cost_time;
    }
    if (this_cost_time > client_stat.max_leave_single_role_time) {
      client_stat.max_leave_single_role_time = this_cost_time;
    }
  }
  end_time = Time::Now().ToNanosecond();
  client_stat.total_leave_roles_time = end_time - start_time;
  client_stat.avg_leave_single_role_time =
      client_stat.total_leave_roles_time / client_num;

  // Show results
  std::cout << "********** ServiceManager Run Statistics **********"
            << std::endl;
  std::cout << "***** Service *****" << std::endl;
  service_stat.Display();
  std::cout << "******************" << std::endl;
  std::cout << "***** Client *****" << std::endl;
  client_stat.Display();
  std::cout << "******************" << std::endl;
  std::cout << "************************************************" << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc > 6) {
    Usage();
    return -1;
  }

  RunConfig cfg;
  Parse(argc, argv, &cfg);
  cfg.Display();

  Topology::Instance();
  RunNodeTest(cfg.role_num[TOPO_NODE]);
  RunChannelTest(cfg.role_num[TOPO_WRITER], cfg.role_num[TOPO_READER]);
  RunServiceTest(cfg.role_num[TOPO_SERVICE], cfg.role_num[TOPO_CLIENT]);
  Topology::Instance()->Shutdown();
  return 0;
}
