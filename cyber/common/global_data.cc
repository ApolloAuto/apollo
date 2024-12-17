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

#include "cyber/common/global_data.h"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdlib>
#include <functional>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"

namespace apollo {
namespace cyber {
namespace common {

AtomicHashMap<uint64_t, std::string, 512> GlobalData::node_id_map_;
AtomicHashMap<uint64_t, std::string, 256> GlobalData::channel_id_map_;
AtomicHashMap<uint64_t, std::string, 256> GlobalData::service_id_map_;
AtomicHashMap<uint64_t, std::string, 256> GlobalData::task_id_map_;

namespace {
const std::string& kEmptyString = "";
std::string program_path() {
  char path[PATH_MAX];
  auto len = readlink("/proc/self/exe", path, sizeof(path) - 1);
  if (len == -1) {
    return kEmptyString;
  }
  path[len] = '\0';
  return std::string(path);
}
}  // namespace

GlobalData::GlobalData() {
  InitHostInfo();
  ACHECK(InitConfig());
  process_id_ = getpid();
  auto prog_path = program_path();
  if (!prog_path.empty()) {
    process_group_ = GetFileName(prog_path) + "_" + std::to_string(process_id_);
  } else {
    process_group_ = "cyber_default_" + std::to_string(process_id_);
  }

  const auto& run_mode_conf = config_.run_mode_conf();
  run_mode_ = run_mode_conf.run_mode();
  clock_mode_ = run_mode_conf.clock_mode();
}

GlobalData::~GlobalData() {}

int GlobalData::ProcessId() const { return process_id_; }

void GlobalData::SetProcessGroup(const std::string& process_group) {
  process_group_ = process_group;
}
const std::string& GlobalData::ProcessGroup() const { return process_group_; }

void GlobalData::SetComponentNums(const int component_nums) {
  component_nums_ = component_nums;
}
int GlobalData::ComponentNums() const { return component_nums_; }

void GlobalData::SetSchedName(const std::string& sched_name) {
  sched_name_ = sched_name;
}
const std::string& GlobalData::SchedName() const { return sched_name_; }

const std::string& GlobalData::HostIp() const { return host_ip_; }

const std::string& GlobalData::HostName() const { return host_name_; }

void GlobalData::EnableSimulationMode() {
  run_mode_ = RunMode::MODE_SIMULATION;
}

void GlobalData::DisableSimulationMode() { run_mode_ = RunMode::MODE_REALITY; }

bool GlobalData::IsRealityMode() const {
  return run_mode_ == RunMode::MODE_REALITY;
}

bool GlobalData::IsMockTimeMode() const {
  return clock_mode_ == ClockMode::MODE_MOCK;
}

bool GlobalData::IsChannelEnableArenaShm(std::string channel_name) const {
  if (!config_.has_transport_conf() ||
      !config_.transport_conf().has_shm_conf() ||
      !config_.transport_conf().shm_conf().has_arena_shm_conf()) {
    return false;
  }
  bool found = false;
  for (auto arena_channel_conf : config_.transport_conf()
                                     .shm_conf()
                                     .arena_shm_conf()
                                     .arena_channel_conf()) {
    if (channel_name == arena_channel_conf.channel_name()) {
      found = true;
      break;
    }
  }
  return found;
}

bool GlobalData::IsChannelEnableArenaShm(uint64_t channel_id) const {
  auto channel_name =
      cyber::common::GlobalData::Instance()->GetChannelById(channel_id);
  return IsChannelEnableArenaShm(channel_name);
}

apollo::cyber::proto::ArenaChannelConf GlobalData::GetChannelArenaConf(
    std::string channel_name) const& {
  for (auto arena_channel_conf : config_.transport_conf()
                                     .shm_conf()
                                     .arena_shm_conf()
                                     .arena_channel_conf()) {
    if (channel_name == arena_channel_conf.channel_name()) {
      return arena_channel_conf;
    }
  }
  return apollo::cyber::proto::ArenaChannelConf();
}

apollo::cyber::proto::ArenaChannelConf GlobalData::GetChannelArenaConf(
    uint64_t channel_id) const& {
  auto channel_name =
      cyber::common::GlobalData::Instance()->GetChannelById(channel_id);
  return GetChannelArenaConf(channel_name);
}

void GlobalData::InitHostInfo() {
  char host_name[1024];
  gethostname(host_name, sizeof(host_name));
  host_name_ = host_name;

  host_ip_ = "127.0.0.1";

  // if we have exported a non-loopback CYBER_IP, we will use it firstly,
  // otherwise, we try to find first non-loopback ipv4 addr.
  const char* ip_env = getenv("CYBER_IP");
  if (ip_env != nullptr) {
    // maybe we need to verify ip_env
    std::string ip_env_str(ip_env);
    std::string starts = ip_env_str.substr(0, 3);
    if (starts != "127") {
      host_ip_ = ip_env_str;
      AINFO << "host ip: " << host_ip_;
      return;
    }
  }

  ifaddrs* ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0) {
    AERROR << "getifaddrs failed, we will use 127.0.0.1 as host ip.";
    return;
  }
  for (ifaddrs* ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr) {
      continue;
    }
    int family = ifa->ifa_addr->sa_family;
    if (family != AF_INET) {
      continue;
    }
    char addr[NI_MAXHOST] = {0};
    if (getnameinfo(ifa->ifa_addr, sizeof(sockaddr_in), addr, NI_MAXHOST, NULL,
                    0, NI_NUMERICHOST) != 0) {
      continue;
    }
    std::string tmp_ip(addr);
    std::string starts = tmp_ip.substr(0, 3);
    if (starts != "127") {
      host_ip_ = tmp_ip;
      break;
    }
  }
  freeifaddrs(ifaddr);
  AINFO << "host ip: " << host_ip_;
}

bool GlobalData::InitConfig() {
  auto config_path = GetAbsolutePath(WorkRoot(), "conf/cyber.pb.conf");
  if (!GetProtoFromFile(config_path, &config_)) {
    AERROR << "read cyber default conf failed!";
    return false;
  }

  return true;
}

const CyberConfig& GlobalData::Config() const { return config_; }

uint64_t GlobalData::RegisterNode(const std::string& node_name) {
  auto id = Hash(node_name);
  while (node_id_map_.Has(id)) {
    std::string* name = nullptr;
    node_id_map_.Get(id, &name);
    if (node_name == *name) {
      break;
    }
    ++id;
    AWARN << " Node name hash collision: " << node_name << " <=> " << *name;
  }
  node_id_map_.Set(id, node_name);
  return id;
}

std::string GlobalData::GetNodeById(uint64_t id) {
  std::string* node_name = nullptr;
  if (node_id_map_.Get(id, &node_name)) {
    return *node_name;
  }
  return kEmptyString;
}

uint64_t GlobalData::RegisterChannel(const std::string& channel) {
  auto id = Hash(channel);
  while (channel_id_map_.Has(id)) {
    std::string* name = nullptr;
    channel_id_map_.Get(id, &name);
    if (channel == *name) {
      break;
    }
    ++id;
    AWARN << "Channel name hash collision: " << channel << " <=> " << *name;
  }
  channel_id_map_.Set(id, channel);
  return id;
}

std::string GlobalData::GetChannelById(uint64_t id) {
  std::string* channel = nullptr;
  if (channel_id_map_.Get(id, &channel)) {
    return *channel;
  }
  return kEmptyString;
}

uint64_t GlobalData::RegisterService(const std::string& service) {
  auto id = Hash(service);
  while (service_id_map_.Has(id)) {
    std::string* name = nullptr;
    service_id_map_.Get(id, &name);
    if (service == *name) {
      break;
    }
    ++id;
    AWARN << "Service name hash collision: " << service << " <=> " << *name;
  }
  service_id_map_.Set(id, service);
  return id;
}

std::string GlobalData::GetServiceById(uint64_t id) {
  std::string* service = nullptr;
  if (service_id_map_.Get(id, &service)) {
    return *service;
  }
  return kEmptyString;
}

uint64_t GlobalData::RegisterTaskName(const std::string& task_name) {
  auto id = Hash(task_name);
  while (task_id_map_.Has(id)) {
    std::string* name = nullptr;
    task_id_map_.Get(id, &name);
    if (task_name == *name) {
      break;
    }
    ++id;
    AWARN << "Task name hash collision: " << task_name << " <=> " << *name;
  }
  task_id_map_.Set(id, task_name);
  return id;
}

std::string GlobalData::GetTaskNameById(uint64_t id) {
  std::string* task_name = nullptr;
  if (task_id_map_.Get(id, &task_name)) {
    return *task_name;
  }
  return kEmptyString;
}

}  // namespace common
}  // namespace cyber
}  // namespace apollo
