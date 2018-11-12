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

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::common::GlobalData;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

SchedulerChoreography::SchedulerChoreography() {
  sched_policy_ = SchedPolicy::CHOREO;

  auto gconf = GlobalData::Instance()->Config();
  for (auto& conf : gconf.scheduler_conf().confs()) {
    sched_confs_[conf.name()] = conf;
  }

  auto desc = SchedName_descriptor()->
      FindValueByName(GlobalData::Instance()->SchedName());
  int sname;
  if (desc) {
    sname = desc->number();

    SchedConf sconf;
    auto itr = sched_confs_.find(sname);
    if (itr != sched_confs_.end()) {
      proc_num_ = itr->second.proc_num();
      task_pool_size_ = itr->second.task_pool_size();
      cpu_binding_start_index_ = itr->second.cpu_binding_start_index();
    }
  }

  for (auto& conf : gconf.choreo_conf()) {
    if (conf.sched_name() == sname) {
      for (auto& choreo : conf.choreos()) {
        cr_confs_[choreo.name()] = choreo;
      }
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
    auto ctx = std::make_shared<ChoreoGraphyContext>();

    proc->BindContext(ctx);
    proc->SetBindCpuIndex(cpu_binding_start_index_ + i);
    ctx->BindProc(proc);
    pctxs_.emplace_back(ctx);

    proc->Start();
  }

  // Put tasks w/o processor assigned into a classic pool.
  for (uint32_t i = 0; i < task_pool_size_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ClassicContext>();

    proc->BindContext(ctx);
    proc->SetBindCpuIndex(cpu_binding_start_index_ + proc_num_ + i);
    ctx->BindProc(proc);
    pctxs_.emplace_back(ctx);

    proc->Start();
  }
}

bool SchedulerChoreography::DispatchTask(const std::shared_ptr<CRoutine> cr) {
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

  uint32_t pid = cr->processor_id();
  if (pid >= 0 && pid < proc_num_) {
    {
      WriteLockGuard<AtomicRWLock> lk(cr_ctx_lock_);
      cr_ctx_[cr->id()] = pid;
    }
    return pctxs_[pid]->Enqueue(cr);
  } else {
    // fallback for tasks w/o processor assigned.

    // Check if task prio is resonable.
    if (cr->priority() < 0 || cr->priority() >= MAX_PRIO) {
      return false;
    }

    // Enqueue task.
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
  auto crid = GlobalData::RegisterTaskName(name);
  {
    WriteLockGuard<AtomicRWLock> lk(cr_ctx_lock_);
    auto p = cr_ctx_.find(crid);
    if (p != cr_ctx_.end()) {
      pctxs_[p->second]->RemoveCRoutine(crid);
      cr_ctx_.erase(crid);
    }
  }

  // Remove from pool.
  for (int i = MAX_PRIO - 1; i >= 0; --i) {
    WriteLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[i]);

    for (auto it = ClassicContext::rq_[i].begin();
          it != ClassicContext::rq_[i].end(); ++it) {
      if ((*it)->id() == crid) {
        ClassicContext::rq_[i].erase(it);
        break;
      }
    }
  }

  return true;
}

bool SchedulerChoreography::NotifyProcessor(uint64_t crid) {
  if (unlikely(stop_)) {
    return true;
  }

  // noitify choreography cr & processor
  ReadLockGuard<AtomicRWLock> lk(cr_ctx_lock_);
  auto it = cr_ctx_.find(crid);
  if (it != cr_ctx_.end()) {
    PerfEventCache::Instance()->
        AddSchedEvent(SchedPerf::NOTIFY_IN,
                      crid, it->second);

    pctxs_[it->second]->Notify(crid);
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

