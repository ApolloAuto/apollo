#ifndef apollo_PERCEPTION_ONBOARD_DAG_STREAMING_H
#define apollo_PERCEPTION_ONBOARD_DAG_STREAMING_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"

#include "modules/common/macro.h"
#include "modules/perception/lib/base/thread.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/proto/dag_config.pb.h"
#include "modules/perception/onboard/shared_data_manager.h"

namespace apollo {
namespace perception {

DECLARE_int32(num_threads_in_dag);
DECLARE_int32(max_allowed_congestion_value);
DECLARE_bool(enable_timing_remove_stale_data);

class Subnode;
class DAGStreamingMonitor;

class DAGStreaming : public Thread {
 public:
  DAGStreaming();
  virtual ~DAGStreaming();
  bool init(const std::string& dag_config_path);

  void stop();

  size_t num_subnodes() const {
    return _subnode_map.size();
  }

  void reset();

  void remove_stale_data() {
    _shared_data_manager.remove_stale_data();
  }

  size_t congestion_value() const;

 protected:
  virtual void run() override;

 private:
  DISALLOW_COPY_AND_ASSIGN(DAGStreaming);

  // start run and wait.
  void schedule();
  bool init_subnodes(const DAGConfig& dag_config);
  bool init_shared_data(const DAGConfig::SharedDataConfig& data_config);

  typedef std::map<SubnodeID, std::unique_ptr<Subnode>> SubnodeMap;

  EventManager _event_manager;
  SharedDataManager _shared_data_manager;
  bool _inited;
  std::unique_ptr<DAGStreamingMonitor> _monitor;
  // NOTE(Yangguang Li): Guarantee Sunode should be firstly called destructor.
  // Subnode depends the EventManager and SharedDataManager.
  SubnodeMap _subnode_map;
};

class DAGStreamingMonitor : public Thread {
 public:
  explicit DAGStreamingMonitor(DAGStreaming* dag_streaming)
      : Thread(true, "DAGStreamingMonitor"),
        _dag_streaming(dag_streaming),
        _stop(false) {}

  virtual ~DAGStreamingMonitor() {}

  void stop() {
    _stop = true;
  }

 protected:
  virtual void run() override;

 private:
  DISALLOW_COPY_AND_ASSIGN(DAGStreamingMonitor);
  // Not own DAGStreaming instance.
  DAGStreaming* _dag_streaming;
  volatile bool _stop;
};

}  // namespace perception
}  // namespace apollo

#endif  // apollo_PERCEPTION_ONBOARD_DAG_STREAMING_H
