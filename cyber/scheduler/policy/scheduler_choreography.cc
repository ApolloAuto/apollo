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

#include "cyber/scheduler/policy/scheduler_choreography.h"

#include <memory>
#include <string>

#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/policy/choreography.h"
#include "cyber/scheduler/policy/classic.h"
#include "cyber/common/environment.h"
#include "cyber/common/file.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::common::GlobalData;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;
using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::PathExists;
using apollo::cyber::common::GetProtoFromFile;
using apollo::cyber::common::WorkRoot;

SchedulerChoreography::SchedulerChoreography() {
  // get sched config
  std::string conf("conf/");
  conf.append(GlobalData::Instance()->ProcessGroup()).append(".conf");
  auto cfg_file = GetAbsolutePath(WorkRoot(), conf);

  apollo::cyber::proto::CyberConfig cfg;
  if (PathExists(cfg_file) && GetProtoFromFile(cfg_file, &cfg)) {
    proc_num_ = cfg.scheduler_conf().choreography_conf()
        .choreography_processor_num();
    choreography_affinity_ = cfg.scheduler_conf().choreography_conf()
        .choreography_affinity();
    ParseCpuset(cfg.scheduler_conf().choreography_conf().choreography_cpuset(),
        &choreography_cpuset_);

    task_pool_size_ = cfg.scheduler_conf().choreography_conf()
        .pool_processor_num();
    pool_affinity_ = cfg.scheduler_conf().choreography_conf()
        .pool_affinity();
    ParseCpuset(cfg.scheduler_conf().choreography_conf().pool_cpuset(),
        &pool_cpuset_);

    for (auto& task : cfg.scheduler_conf().choreography_conf().tasks()) {
      cr_confs_[task.name()] = task;
    }
  }

  // default val for case w/o config:
  if (proc_num_ == 0) {
    proc_num_ = std::thread::hardware_concurrency() / 4 * 3;
    task_pool_size_ = std::thread::hardware_concurrency() / 4;
    // FIXME: other vals ... ?
  }

  CreateProcessor();
}

void SchedulerChoreography::CreateProcessor() {
  for (uint32_t i = 0; i < proc_num_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ChoreographyContext>();

    proc->BindContext(ctx);
    proc->SetAffinity(choreography_cpuset_, choreography_affinity_, i);
    ctx->BindProc(proc);
    pctxs_.emplace_back(ctx);
  }

  // Put tasks w/o processor assigned into a classic pool.
  for (uint32_t i = 0; i < task_pool_size_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ClassicContext>();

    proc->BindContext(ctx);
    proc->SetAffinity(pool_cpuset_, pool_affinity_, i);
    ctx->BindProc(proc);
    pctxs_.emplace_back(ctx);
  }
}

bool SchedulerChoreography::DispatchTask(const std::shared_ptr<CRoutine> cr) {
  // we use multi-key mutex to prevent race condition
  // when del && add cr with same crid
  std::lock_guard<std::mutex> lg(cr_del_lock_[cr->id()]);

  // Assign sched cfg to tasks according to configuration.
  if (cr_confs_.find(cr->name()) != cr_confs_.end()) {
    ChoreographyTask taskconf = cr_confs_[cr->name()];
    cr->set_priority(taskconf.prio());

    if (taskconf.has_processor()) {
      cr->set_processor_id(taskconf.processor());
    }
  }

  // Create CRoutine context;
  {
    WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
    if (id_cr_.find(cr->id()) != id_cr_.end()) {
      return false;
    }
    id_cr_[cr->id()] = cr;
  }

  // Enqueue task.
  uint32_t pid = cr->processor_id();
  if (pid < proc_num_) {
    // Enqueue task to Choreo Policy.
    static_cast<ChoreographyContext *>(pctxs_[pid].get())->Enqueue(cr);
  } else {
    // fallback for tasks w/o processor assigned.

    // Check if task prio is reasonable.
    if (cr->priority() >= MAX_PRIO) {
      AWARN << cr->name()
            << " prio great than MAX_PRIO.";
      cr->set_priority(MAX_PRIO - 1);
    }

    // Enqueue task to pool runqueue.
    {
      WriteLockGuard<AtomicRWLock>
          lk(ClassicContext::rq_locks_[cr->priority()]);
      ClassicContext::rq_[cr->priority()].emplace_back(cr);
    }
  }
  return true;
}

bool SchedulerChoreography::RemoveTask(const std::string& name) {
  if (unlikely(stop_)) {
    return true;
  }

  auto crid = GlobalData::GenerateHashId(name);

  // we use multi-key mutex to prevent race condition
  // when del && add cr with same crid
  std::lock_guard<std::mutex> lg(cr_del_lock_[crid]);

  // Find cr from id_cr &&
  // get cr prio if cr found
  int prio;
  int pid;
  {
    ReadLockGuard<AtomicRWLock> lk(id_cr_lock_);
    auto p = id_cr_.find(crid);
    if (p != id_cr_.end()) {
      auto cr = p->second;
      prio = cr->priority();
      pid = cr->processor_id();
    } else {
      return false;
    }
  }

  // rm cr from pool if rt not in choreo context
  if (pid == -1) {
    WriteLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[prio]);
    for (auto it = ClassicContext::rq_[prio].begin();
         it != ClassicContext::rq_[prio].end(); ++it) {
      if ((*it)->id() == crid) {
        ClassicContext::rq_[prio].erase(it);
        return true;
      }
    }
  }

  // Remove cr from id_cr &&
  // Set CRoutine Stop Tag, Choreo policy will
  // rm finished task by it-self
  {
    WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
    if (id_cr_.find(crid) != id_cr_.end()) {
      id_cr_[crid]->Stop();
      id_cr_.erase(crid);
    }
  }
  return false;
}

bool SchedulerChoreography::NotifyProcessor(uint64_t crid) {
  if (unlikely(stop_)) {
    return true;
  }

  std::shared_ptr<CRoutine> cr = nullptr;
  // find cr from id_cr && Update cr Flag
  // policies will handle ready-state CRoutines
  {
    ReadLockGuard<AtomicRWLock> lk(id_cr_lock_);
    auto it = id_cr_.find(crid);
    if (it != id_cr_.end()) {
      cr = it->second;
      if (cr->state() == RoutineState::DATA_WAIT) {
        cr->SetUpdateFlag();
      }
    } else {
      return false;
    }
  }

  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NOTIFY_IN, crid,
                                            cr->processor_id());

  // notify processor in choreo context
  if (cr->processor_id() != -1) {
    auto pid = cr->processor_id();
    static_cast<ChoreographyContext*>(pctxs_[pid].get())->Notify();
  }

  return true;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
