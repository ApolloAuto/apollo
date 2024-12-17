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

#ifndef CYBER_COMMON_GLOBAL_DATA_H_
#define CYBER_COMMON_GLOBAL_DATA_H_

#include <string>
#include <unordered_map>

#include "cyber/proto/cyber_conf.pb.h"
#include "cyber/proto/transport_conf.pb.h"

#include "cyber/base/atomic_hash_map.h"
#include "cyber/base/atomic_rw_lock.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/common/util.h"

namespace apollo {
namespace cyber {
namespace common {

using ::apollo::cyber::base::AtomicHashMap;
using ::apollo::cyber::proto::ClockMode;
using ::apollo::cyber::proto::CyberConfig;
using ::apollo::cyber::proto::RunMode;

class GlobalData {
 public:
  ~GlobalData();

  int ProcessId() const;

  void SetProcessGroup(const std::string& process_group);
  const std::string& ProcessGroup() const;

  void SetComponentNums(const int component_nums);
  int ComponentNums() const;

  void SetSchedName(const std::string& sched_name);
  const std::string& SchedName() const;

  const std::string& HostIp() const;

  const std::string& HostName() const;

  const CyberConfig& Config() const;

  void EnableSimulationMode();
  void DisableSimulationMode();

  bool IsRealityMode() const;
  bool IsMockTimeMode() const;

  bool IsChannelEnableArenaShm(std::string channel_name) const;
  bool IsChannelEnableArenaShm(uint64_t channel_id) const;
  apollo::cyber::proto::ArenaChannelConf GetChannelArenaConf(
      std::string channel_name) const&;
  apollo::cyber::proto::ArenaChannelConf GetChannelArenaConf(
      uint64_t channel_id) const&;

  static uint64_t GenerateHashId(const std::string& name) {
    return common::Hash(name);
  }

  static uint64_t RegisterNode(const std::string& node_name);
  static std::string GetNodeById(uint64_t id);

  static uint64_t RegisterChannel(const std::string& channel);
  static std::string GetChannelById(uint64_t id);

  static uint64_t RegisterService(const std::string& service);
  static std::string GetServiceById(uint64_t id);

  static uint64_t RegisterTaskName(const std::string& task_name);
  static std::string GetTaskNameById(uint64_t id);

 private:
  void InitHostInfo();
  bool InitConfig();

  // global config
  CyberConfig config_;

  // host info
  std::string host_ip_;
  std::string host_name_;

  // process info
  int process_id_;
  std::string process_group_;

  int component_nums_ = 0;

  // sched policy info
  std::string sched_name_ = "CYBER_DEFAULT";

  // run mode
  RunMode run_mode_;
  ClockMode clock_mode_;

  static AtomicHashMap<uint64_t, std::string, 512> node_id_map_;
  static AtomicHashMap<uint64_t, std::string, 256> channel_id_map_;
  static AtomicHashMap<uint64_t, std::string, 256> service_id_map_;
  static AtomicHashMap<uint64_t, std::string, 256> task_id_map_;

  DECLARE_SINGLETON(GlobalData)
};

}  // namespace common
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_COMMON_GLOBAL_DATA_H_
