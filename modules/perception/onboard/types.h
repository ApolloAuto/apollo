#ifndef apollo_PERCEPTION_ONBOARD_TYPES_H
#define apollo_PERCEPTION_ONBOARD_TYPES_H

#include <sstream>
#include <string>

#include "modules/perception/lib/base/concurrent_queue.h"
#include "modules/perception/lib/base/time_util.h"
#include "modules/perception/common/define.h"

namespace apollo {
namespace perception {

using EventID = int;
using SubnodeID = int;

struct Event {
  EventID event_id = 0;
  double timestamp = 0.0;
  std::string reserve;
  // TODO(Yangguang Li):
  double local_timestamp = 0.0;  // local timestamp to compute process delay.

  Event() {
    local_timestamp = TimeUtil::get_current_time();
  }

  std::string to_string() const {
    std::ostringstream oss;
    oss << "event_id: " << event_id
        << " timestamp: " << GLOG_TIMESTAMP(timestamp)
        << " reserve: " << reserve;
    return oss.str();
  }
};

struct EventMeta {
  EventID event_id;
  std::string name;
  SubnodeID from_node;
  SubnodeID to_node;
  EventMeta() : event_id(0), from_node(0), to_node(0) {}

  std::string to_string() const {
    std::ostringstream oss;
    oss << "event_id: " << event_id << " name: " << name
        << " from_node: " << from_node << " to_node: " << to_node;
    return oss.str();
  }
};

enum IoStreamType {
  // source type
  ROSMSG_SOURCE = 0,
  FILE_SOURCE = 1,
  PCD_FILE_SOURCE = 2,
  // sink type
  ROSMSG_SINK = 9,
  FILE_SINK = 10,
  BENCHMARK_FILE_SINK = 11,
  // unknown
  UNKNOWN_TYPE = 16,
};

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_ONBOARD_TYPES_H
