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
    task_pool_size_ = cfg.scheduler_conf().choreography_conf().
                        pool_processor_num();
    // FIXME: pool cpuset
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
    ctx->BindProc(proc);
    pctxs_.emplace_back(ctx);

    proc->Start();
  }

  // Put tasks w/o processor assigned into a classic pool.
  for (uint32_t i = 0; i < task_pool_size_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ClassicContext>();

    proc->BindContext(ctx);
    ctx->BindProc(proc);
    pctxs_.emplace_back(ctx);

    proc->Start();
  }
}

bool SchedulerChoreography::DispatchTask(const std::shared_ptr<CRoutine> cr) {
  // Assign sched cfg to tasks according to configuration.
  if (cr_confs_.find(cr->name()) != cr_confs_.end()) {
    ChoreographyTask taskconf = cr_confs_[cr->name()];
    cr->set_priority(taskconf.prio());

    if (taskconf.has_processor()) {
      cr->set_processor_id(taskconf.processor());
    }
  }

  // Check if task exists in choreography processor rq already.
  {
    WriteLockGuard<AtomicRWLock> lk(cr_ctx_lock_);
    if (cr_ctx_.find(cr->id()) != cr_ctx_.end()) {
      return false;
    }
  }

  // Check if task exists in pool already.
  for (int i = MAX_PRIO - 1; i >= 0; --i) {
    ReadLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[i]);

    for (auto it = ClassicContext::rq_[i].begin();
         it != ClassicContext::rq_[i].end(); ++it) {
      if ((*it)->id() == cr->id()) {
        return false;
      }
    }
  }

  // Enqueue task.
  uint32_t pid = cr->processor_id();
  if (pid < proc_num_) {
    {
      WriteLockGuard<AtomicRWLock> lk(cr_ctx_lock_);
      cr_ctx_[cr->id()] = pid;
    }
    // Enqueue task to processor runqueue.
    return static_cast<ChoreographyContext *>(pctxs_[pid].get())->Enqueue(cr);
  } else {
    // fallback for tasks w/o processor assigned.

    // Check if task prio is reasonable.
    if (cr->priority() >= MAX_PRIO) {
      AWARN << cr->name()
            << " prio great than MAX_PRIO.";
      cr->set_priority(MAX_PRIO - 1);
    }

    // Enqueue task to pool runqueue.
    WriteLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[cr->priority()]);
    ClassicContext::rq_[cr->priority()].emplace_back(cr);
  }

  return true;
}

bool SchedulerChoreography::RemoveTask(const std::string& name) {
  if (unlikely(stop_)) {
    return true;
  }

  // Remove from processor's runqueue
  auto crid = GlobalData::GenerateHashId(name);
  {
    WriteLockGuard<AtomicRWLock> lk(cr_ctx_lock_);
    auto p = cr_ctx_.find(crid);
    if (p != cr_ctx_.end()) {
      static_cast<ChoreographyContext*>(pctxs_[p->second].get())
          ->RemoveCRoutine(crid);
      cr_ctx_.erase(crid);
      return true;
    }
  }

  // Remove from pool.
  for (int i = MAX_PRIO - 1; i >= 0; --i) {
    WriteLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[i]);

    for (auto it = ClassicContext::rq_[i].begin();
         it != ClassicContext::rq_[i].end(); ++it) {
      if ((*it)->id() == crid) {
        (*it)->Stop();
        ClassicContext::rq_[i].erase(it);
        return true;
      }
    }
  }

  return false;
}

bool SchedulerChoreography::NotifyProcessor(uint64_t crid) {
  if (unlikely(stop_)) {
    return true;
  }

  // noitify choreography cr & processor
  ReadLockGuard<AtomicRWLock> lk(cr_ctx_lock_);
  auto it = cr_ctx_.find(crid);
  if (it != cr_ctx_.end()) {
    PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NOTIFY_IN, crid,
                                              it->second);

    static_cast<ChoreographyContext*>(pctxs_[it->second].get())->Notify(crid);

    return true;
  }

  // notify pool cr & processor
  for (int i = MAX_PRIO - 1; i >= 0; --i) {
    ReadLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[i]);

    for (auto it = ClassicContext::rq_[i].begin();
         it != ClassicContext::rq_[i].end(); ++it) {
      if ((*it)->id() == crid) {
        if ((*it)->state() == RoutineState::DATA_WAIT) {
          (*it)->SetUpdateFlag();
        }
        // FIXME: notify processor.
        return true;
      }
    }
  }

  return false;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
