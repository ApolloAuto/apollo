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

#ifndef CYBER_SCHEDULER_SCHEDULER_H_
#define CYBER_SCHEDULER_SCHEDULER_H_

#include <unistd.h>

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/base/atomic_rw_lock.h"
#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/common/types.h"
#include "cyber/croutine/croutine.h"
#include "cyber/croutine/routine_factory.h"
#include "cyber/proto/croutine_conf.pb.h"
#include "cyber/proto/scheduler_conf.pb.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::proto::CRoutineConf;
using apollo::cyber::proto::SchedulerConf;
using apollo::cyber::croutine::CRoutine;
using apollo::cyber::croutine::RoutineFactory;
using apollo::cyber::data::DataVisitorBase;
using apollo::cyber::proto::ProcessStrategy;

using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;

class ProcessorContext;

class Scheduler {
 public:
  ~Scheduler();
  bool CreateTask(const RoutineFactory& factory, const std::string& name);
  bool CreateTask(std::function<void()>&& func, const std::string& name,
                  std::shared_ptr<DataVisitorBase> visitor = nullptr);
  bool DispatchTask(const std::shared_ptr<CRoutine>& croutine);
  bool RemoveTask(const std::string& name);
  bool RemoveCRoutine(uint64_t cr_id);

  bool NotifyProcessor(uint64_t cr_id);
  bool NotifyTask(uint64_t task_id);

  void ShutDown();

  uint32_t ProcessorNum() { return proc_num_; }
  inline std::unordered_map<uint64_t, uint32_t> RtCtx() { return cr_ctx_; }
  inline std::vector<std::shared_ptr<ProcessorContext>> ProcCtxs() {
    return proc_ctxs_;
  }

 private:
  Scheduler(Scheduler&) = delete;
  Scheduler& operator=(Scheduler&) = delete;

  void CreateProcessor();
  std::shared_ptr<ProcessorContext> FindProc(const std::shared_ptr<CRoutine>&);

  void StartSysmon();

  std::thread sysmon_;

  SchedulerConf sched_conf_;
  std::unordered_map<std::string, CRoutineConf> cr_confs_;

  ProcessStrategy sched_policy_ = ProcessStrategy::CHOREO;

  AtomicRWLock rw_lock_;
  std::unordered_map<uint64_t, uint32_t> cr_ctx_;
  std::vector<std::shared_ptr<ProcessorContext>> proc_ctxs_;

  uint32_t proc_num_;
  uint32_t task_pool_size_;

  std::atomic<bool> stop_;

  DECLARE_SINGLETON(Scheduler)
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_SCHEDULER_H_
