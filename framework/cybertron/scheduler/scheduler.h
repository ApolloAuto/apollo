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

#ifndef CYBERTRON_SCHEDULER_SCHEDULER_H_
#define CYBERTRON_SCHEDULER_SCHEDULER_H_

#include <unistd.h>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cybertron/common/log.h"
#include "cybertron/common/macros.h"
#include "cybertron/common/types.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/proto/routine_conf.pb.h"
#include "cybertron/proto/scheduler_conf.pb.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::proto::RoutineConf;
using apollo::cybertron::proto::SchedulerConf;
using apollo::cybertron::croutine::CRoutine;
using apollo::cybertron::croutine::RoutineFactory;
using apollo::cybertron::data::DataVisitorBase;
using apollo::cybertron::proto::ProcessStrategy;

class ProcessorContext;

class Scheduler {
 public:
  ~Scheduler();
  bool CreateTask(const RoutineFactory& factory, const std::string& name);
  bool CreateTask(std::function<void()>&& func, const std::string& name,
                  std::shared_ptr<DataVisitorBase> visitor = nullptr);
  bool RemoveTask(const std::string& name);
  void PrintStatistics();
  void ShutDown();
  uint32_t ProcessorNum() { return proc_num_; }

  bool DispatchTask(const std::shared_ptr<CRoutine>& croutine);
  bool RemoveCRoutine(uint64_t cr_id);
  bool NotifyProcessor(uint64_t cr_id) const;
  bool NotifyTask(uint64_t task_id) const;

  inline std::vector<std::shared_ptr<ProcessorContext>> ProcCtxs() {
    return proc_ctxs_;
  }
  inline std::unordered_map<uint64_t, uint32_t> RtCtx() { return rt_ctx_; }
  inline bool IsClassic() { return classic_; }

 private:
  Scheduler(Scheduler&) = delete;
  Scheduler& operator=(Scheduler&) = delete;
  void CreateProcessor();
  std::shared_ptr<ProcessorContext> FindProc(const std::shared_ptr<CRoutine>&);
  void StartSysmon();

  std::atomic<bool> stop_;
  std::thread sysmon_;
  SchedulerConf sched_conf_;
  RoutineConf rt_conf_;
  std::mutex task_id_map_mutex_;
  std::unordered_map<uint64_t, std::string> task_id_map_;
  uint32_t proc_num_;
  ProcessStrategy sched_policy_ = ProcessStrategy::CFS;
  std::vector<std::shared_ptr<ProcessorContext>> proc_ctxs_;
  std::mutex mtx_ctx_qsize_;
  std::multimap<int, std::shared_ptr<ProcessorContext>> ctx_qsize_;
  std::unordered_map<uint64_t, uint32_t> rt_ctx_;
  bool classic_ = false;

  DECLARE_SINGLETON(Scheduler)
};

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_SCHEDULER_H_
