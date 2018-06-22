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

#ifndef MODULES_PERCEPTION_ONBOARD_DAG_STREAMING_H_
#define MODULES_PERCEPTION_ONBOARD_DAG_STREAMING_H_

#include <map>
#include <memory>
#include <string>
// #include <unordered_map>
#include <vector>

#include "gflags/gflags.h"

#include "modules/perception/onboard/proto/dag_config.pb.h"

#include "modules/common/macro.h"
#include "modules/perception/lib/base/thread.h"
#include "modules/perception/onboard/event_manager.h"
#include "modules/perception/onboard/shared_data_manager.h"

namespace apollo {
namespace perception {

DECLARE_int32(num_threads_in_dag);
DECLARE_int32(max_allowed_congestion_value);
DECLARE_bool(enable_timing_remove_stale_data);

class Subnode;
class DAGStreamingMonitor;

typedef std::map<SubnodeID, std::unique_ptr<Subnode>> SubnodeMap;

class DAGStreaming : public Thread {
 public:
  DAGStreaming();
  virtual ~DAGStreaming();

  bool Init(const std::string &dag_config_path);

  void Stop();

  size_t NumSubnodes() const { return subnode_map_.size(); }

  void Reset();

  void RemoveStaleData() { shared_data_manager_.RemoveStaleData(); }

  size_t CongestionValue() const;

  static Subnode *GetSubnodeByName(std::string name);

 protected:
  void Run() override;

 private:
  // start run and wait.
  void Schedule();

  bool InitSubnodes(const DAGConfig &dag_config);

  bool InitSharedData(const DAGConfig::SharedDataConfig &data_config);

  EventManager event_manager_;
  SharedDataManager shared_data_manager_;
  bool inited_ = false;
  std::unique_ptr<DAGStreamingMonitor> monitor_;
  // NOTE(Yangguang Li): Guarantee Sunode should be firstly called destructor.
  // Subnode depends the EventManager and SharedDataManager.
  static SubnodeMap subnode_map_;

  // subnode has order, IDs define the initilization order
  static std::map<std::string, SubnodeID> subnode_name_map_;

  DISALLOW_COPY_AND_ASSIGN(DAGStreaming);
};

class DAGStreamingMonitor : public Thread {
 public:
  explicit DAGStreamingMonitor(DAGStreaming *dag_streaming)
      : Thread(true, "DAGStreamingMonitor"),
        dag_streaming_(dag_streaming),
        stop_(false) {}

  virtual ~DAGStreamingMonitor() {}

  void Stop() { stop_ = true; }

 protected:
  void Run() override;

 private:
  // Not own DAGStreaming instance.
  DAGStreaming *dag_streaming_;
  volatile bool stop_;
  DISALLOW_COPY_AND_ASSIGN(DAGStreamingMonitor);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_ONBOARD_DAG_STREAMING_H_
