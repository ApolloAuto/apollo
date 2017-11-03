#include "modules/perception/onboard/dag_streaming.h"

#include <unistd.h>
#include <map>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "google/protobuf/text_format.h"

#include "modules/common/log.h"
#include "modules/perception/lib/base/file_util.h"
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

DAGStreaming::DAGStreaming()
    : Thread(true, "DAGStreamingThread"),
      _inited(false),
      _monitor(new DAGStreamingMonitor(this)) {}

DAGStreaming::~DAGStreaming() {}

bool DAGStreaming::init(const string& dag_config_path) {
  if (_inited) {
    AWARN << "DAGStreaming init twice.";
    return true;
  }

  DAGConfig dag_config;
  string content;
  if (!FileUtil::GetFileContent(dag_config_path, &content)) {
    AERROR << "failed to laod DAGConfig file: " << dag_config_path;
    return false;
  }

  if (!TextFormat::ParseFromString(content, &dag_config)) {
    AERROR << "failed to Parse DAGConfig proto: " << dag_config_path;
    return false;
  }

  if (!_event_manager.init(dag_config.edge_config())) {
    AERROR << "failed to init EventManager. file: " << dag_config_path;
    return false;
  }

  if (!init_shared_data(dag_config.data_config()) ||
      !init_subnodes(dag_config)) {
    return false;
  }

  _inited = true;
  AINFO << "DAGStreaming init success.";
  return true;
}

void DAGStreaming::schedule() {
  _monitor->start();
  // start all subnodes.
  for (auto& pair : _subnode_map) {
    pair.second->start();
  }

  AINFO << "DAGStreaming start to schedule...";

  for (auto& pair : _subnode_map) {
    pair.second->join();
  }

  _monitor->join();
  AINFO << "DAGStreaming schedule exit.";
}

void DAGStreaming::stop() {
  _monitor->stop();
  // stop all subnodes.
  for (auto& pair : _subnode_map) {
    pair.second->stop();
  }

  // sleep 100 ms
  usleep(100000);
  // kill thread which is blocked
  for (auto& pair : _subnode_map) {
    if (pair.second->is_alive()) {
      AINFO << "pthread_cancel to thread " << pair.second->tid();
      pthread_cancel(pair.second->tid());
    }
  }

  AINFO << "DAGStreaming is stoped.";
}

bool DAGStreaming::init_subnodes(const DAGConfig& dag_config) {
  const DAGConfig::SubnodeConfig& subnode_config = dag_config.subnode_config();
  const DAGConfig::EdgeConfig& edge_config = dag_config.edge_config();

  map<SubnodeID, DAGConfig::Subnode> subnode_config_map;
  map<SubnodeID, vector<EventID>> subnode_sub_events_map;
  map<SubnodeID, vector<EventID>> subnode_pub_events_map;

  for (auto& subnode_proto : subnode_config.subnodes()) {
    std::pair<map<SubnodeID, DAGConfig::Subnode>::iterator, bool> result =
        subnode_config_map.insert(
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

    if (inst == NULL) {
      AERROR << "failed to get subnode instance. name: "
             << subnode_config.name();
      return false;
    }

    bool result = inst->init(
        subnode_config, &_event_manager, &_shared_data_manager,
        subnode_sub_events_map[subnode_id], subnode_pub_events_map[subnode_id]);
    if (!result) {
      AERROR << "failed to init subnode. name: " << inst->name();
      return false;
    }
    _subnode_map.emplace(subnode_id, std::unique_ptr<Subnode>(inst));

    AINFO << "init subnode succ. " << inst->debug_string();
  }

  AINFO << "DAGStreaming load " << _subnode_map.size() << " subnodes, "
        << _event_manager.num_events() << " events.";
  return true;
}

bool DAGStreaming::init_shared_data(
    const DAGConfig::SharedDataConfig& data_config) {
  return _shared_data_manager.init(data_config);
}

void DAGStreaming::run() {
  schedule();
}

void DAGStreaming::reset() {
  _event_manager.reset();
  _shared_data_manager.reset();
  AINFO << "DAGStreaming RESET.";
}

size_t DAGStreaming::congestion_value() const {
  return _event_manager.max_len_of_event_queues();
}

void DAGStreamingMonitor::run() {
  if (FLAGS_max_allowed_congestion_value == 0 &&
      !FLAGS_enable_timing_remove_stale_data) {
    AINFO << "disable to check DAGStreaming congestion value,"
          << "disable timing delete stale shared data.";
    return;
  }

  while (!_stop) {
    if (FLAGS_max_allowed_congestion_value > 0) {
      // Timing to check DAGStreaming congestion value.
      int congestion_value = _dag_streaming->congestion_value();
      if (congestion_value > FLAGS_max_allowed_congestion_value) {
        _dag_streaming->reset();
        AERROR << "DAGStreaming has CONGESTION, reset it."
               << " congestion_value: " << congestion_value
               << " max_allowed_congestion_value: "
               << FLAGS_max_allowed_congestion_value;
      }
    }

    if (FLAGS_enable_timing_remove_stale_data) {
      _dag_streaming->remove_stale_data();
    }
    sleep(1);
  }
}

}  // namespace perception
}  // namespace apollo
