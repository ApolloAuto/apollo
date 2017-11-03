#ifndef MODEULES_PERCEPTION_ONBOARD_DAG_STREAMING_H_
#define MODEULES_PERCEPTION_ONBOARD_DAG_STREAMING_H_

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

  bool Init(const std::string &dag_config_path);

  void Stop();

  size_t NumSubnodes() const {
    return subnode_map_.size();
  }

  void Reset();

  void RemoveStaleData() {
    shared_data_manager_.RemoveStaleData();
  }

  size_t CongestionValue() const;

 protected:
  virtual void Run() override;

 private:
  DISALLOW_COPY_AND_ASSIGN(DAGStreaming);

  // start run and wait.
  void Schedule();

  bool InitSubnodes(const DAGConfig &dag_config);

  bool InitSharedData(const DAGConfig::SharedDataConfig &data_config);

  typedef std::map<SubnodeID, std::unique_ptr<Subnode>> SubnodeMap;

  EventManager event_manager_;
  SharedDataManager shared_data_manager_;
  bool inited_;
  std::unique_ptr<DAGStreamingMonitor> monitor_;
  // NOTE(Yangguang Li): Guarantee Sunode should be firstly called destructor.
  // Subnode depends the EventManager and SharedDataManager.
  SubnodeMap subnode_map_;
};

class DAGStreamingMonitor : public Thread {
 public:
  explicit DAGStreamingMonitor(DAGStreaming* dag_streaming)
      : Thread(true, "DAGStreamingMonitor"),
        dag_streaming_(dag_streaming),
        stop_(false) {
  }

  virtual ~DAGStreamingMonitor() {}

  void Stop() {
    stop_ = true;
  }

 protected:
  virtual void Run() override;

 private:
  DISALLOW_COPY_AND_ASSIGN(DAGStreamingMonitor);
  // Not own DAGStreaming instance.
  DAGStreaming *dag_streaming_;
  volatile bool stop_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODEULES_PERCEPTION_ONBOARD_DAG_STREAMING_H_
