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

#include "modules/perception/onboard/dag_streaming.h"

#include <unistd.h>

#include <utility>

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/perception/onboard/subnode.h"

namespace apollo {
namespace perception {

using std::string;
using std::map;
using std::vector;

using google::protobuf::TextFormat;

DEFINE_int32(max_allowed_congestion_value, 0,
             "When DAGStreaming event_queues max length greater than "
             "max_allowed_congestion_value, reset DAGStreaming."
             "(default is 0, disable this feature.)");
DEFINE_bool(enable_timing_remove_stale_data, true,
            "whether timing clean shared data");

SubnodeMap DAGStreaming::subnode_map_;
std::map<std::string, SubnodeID> DAGStreaming::subnode_name_map_;

DAGStreaming::DAGStreaming()
    : Thread(true, "DAGStreamingThread"),
      inited_(false),
      monitor_(new DAGStreamingMonitor(this)) {}

DAGStreaming::~DAGStreaming() {}

bool DAGStreaming::Init(const string& dag_config_path) {
  if (inited_) {
    AWARN << "DAGStreaming Init twice.";
    return true;
  }

  DAGConfig dag_config;
  string content;
  if (!apollo::common::util::GetContent(dag_config_path, &content)) {
    AERROR << "failed to laod DAGConfig file: " << dag_config_path;
    return false;
  }

  if (!TextFormat::ParseFromString(content, &dag_config)) {
    AERROR << "failed to Parse DAGConfig proto: " << dag_config_path;
    return false;
  }

  if (!event_manager_.Init(dag_config.edge_config())) {
    AERROR << "failed to Init EventManager. file: " << dag_config_path;
    return false;
  }

  if (!InitSharedData(dag_config.data_config()) || !InitSubnodes(dag_config)) {
    return false;
  }

  inited_ = true;
  AINFO << "DAGStreaming Init success.";
  return true;
}

void DAGStreaming::Schedule() {
  monitor_->Start();
  // start all subnodes.
  for (auto& pair : subnode_map_) {
    pair.second->Start();
  }

  AINFO << "DAGStreaming start to schedule...";

  for (auto& pair : subnode_map_) {
    pair.second->Join();
  }

  monitor_->Join();
  AINFO << "DAGStreaming schedule exit.";
}

void DAGStreaming::Stop() {
  monitor_->Stop();
  // stop all subnodes.
  for (auto& pair : subnode_map_) {
    pair.second->Stop();
  }

  // sleep 100 ms
  usleep(100000);
  // kill thread which is blocked
  for (auto& pair : subnode_map_) {
    if (pair.second->IsAlive()) {
      AINFO << "pthread_cancel to thread " << pair.second->Tid();
      pthread_cancel(pair.second->Tid());
    }
  }

  AINFO << "DAGStreaming is stoped.";
}

bool DAGStreaming::InitSubnodes(const DAGConfig& dag_config) {
  const DAGConfig::SubnodeConfig& subnode_config = dag_config.subnode_config();
  const DAGConfig::EdgeConfig& edge_config = dag_config.edge_config();

  map<SubnodeID, DAGConfig::Subnode> subnode_config_map;
  map<SubnodeID, vector<EventID>> subnode_sub_events_map;
  map<SubnodeID, vector<EventID>> subnode_pub_events_map;

  for (auto& subnode_proto : subnode_config.subnodes()) {
    std::pair<map<SubnodeID, DAGConfig::Subnode>::iterator, bool>
        result = subnode_config_map.insert(
            std::make_pair(subnode_proto.id(), subnode_proto));
    if (!result.second) {
      AERROR << "duplicate SubnodeID: " << subnode_proto.id();
      return false;
    }
  }

  for (auto& edge_proto : edge_config.edges()) {
    SubnodeID from = edge_proto.from_node();
    SubnodeID to = edge_proto.to_node();

    if (subnode_config_map.find(from) == subnode_config_map.end() ||
        subnode_config_map.find(to) == subnode_config_map.end()) {
      AERROR << "SubnodeID not exists in subnode_config. <" << from << ", "
             << to << ">";
      return false;
    }

    for (auto& event_proto : edge_proto.events()) {
      subnode_pub_events_map[from].push_back(event_proto.id());
      subnode_sub_events_map[to].push_back(event_proto.id());
    }
  }

  // Generate Subnode instance.
  for (auto pair : subnode_config_map) {
    const DAGConfig::Subnode& subnode_config = pair.second;
    const SubnodeID subnode_id = pair.first;
    Subnode* inst = SubnodeRegisterer::GetInstanceByName(subnode_config.name());

//    AINFO << "subnode_name: " << subnode_config.name();
//    AINFO << "subnode_id: " << subnode_id;
    if (inst == NULL) {
      AERROR << "failed to get subnode instance. name: "
             << subnode_config.name();
      return false;
    }

    bool result = inst->Init(subnode_config, subnode_sub_events_map[subnode_id],
                             subnode_pub_events_map[subnode_id],
                             &event_manager_, &shared_data_manager_);
    if (!result) {
      AERROR << "failed to Init subnode. name: " << inst->name();
      return false;
    }
    subnode_map_.emplace(subnode_id, std::unique_ptr<Subnode>(inst));
    subnode_name_map_[subnode_config.name()] = subnode_id;
    AINFO << "Init subnode succ. " << inst->DebugString();
  }

  AINFO << "DAGStreaming load " << subnode_map_.size() << " subnodes, "
        << event_manager_.NumEvents() << " events.";
  return true;
}

bool DAGStreaming::InitSharedData(
    const DAGConfig::SharedDataConfig& data_config) {
  return shared_data_manager_.Init(data_config);
}

void DAGStreaming::Run() { Schedule(); }

void DAGStreaming::Reset() {
  event_manager_.Reset();
  shared_data_manager_.Reset();
  AINFO << "DAGStreaming RESET.";
}

size_t DAGStreaming::CongestionValue() const {
  return event_manager_.MaxLenOfEventQueues();
}

void DAGStreamingMonitor::Run() {
  if (FLAGS_max_allowed_congestion_value == 0 &&
      !FLAGS_enable_timing_remove_stale_data) {
    AINFO << "disable to check DAGStreaming congestion value,"
          << "disable timing delete stale shared data.";
    return;
  }

  while (!stop_) {
    if (FLAGS_max_allowed_congestion_value > 0) {
      // Timing to check DAGStreaming congestion value.
      int congestion_value = dag_streaming_->CongestionValue();
      if (congestion_value > FLAGS_max_allowed_congestion_value) {
        dag_streaming_->Reset();
        AERROR << "DAGStreaming has CONGESTION, reset it."
               << " congestion_value: " << congestion_value
               << " max_allowed_congestion_value: "
               << FLAGS_max_allowed_congestion_value;
      }
    }

    if (FLAGS_enable_timing_remove_stale_data) {
      dag_streaming_->RemoveStaleData();
    }
    sleep(1);
  }
}

Subnode* DAGStreaming::GetSubnodeByName(std::string name) {
  std::map<std::string, SubnodeID>::iterator iter =
      subnode_name_map_.find(name);
  if (iter != subnode_name_map_.end()) {
    return subnode_map_[iter->second].get();
  }
  return nullptr;
}

}  // namespace perception
}  // namespace apollo
