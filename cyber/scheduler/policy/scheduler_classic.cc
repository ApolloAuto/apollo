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

#include "cyber/scheduler/policy/scheduler_classic.h"

#include <memory>

#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/policy/classic.h"
#include "cyber/event/perf_event_cache.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;
using apollo::cyber::common::GlobalData;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

SchedulerClassic::SchedulerClassic() {
  sched_policy_ = SchedPolicy::CLASSIC;

  auto gconf = GlobalData::Instance()->Config();
  for (auto& conf : gconf.scheduler_conf().confs()) {
    sched_confs_[conf.name()] = conf;
  }

  auto desc = SchedName_descriptor()->
      FindValueByName(GlobalData::Instance()->SchedName());
  if (desc) {
    int sname = desc->number();
    SchedConf sconf;
    auto itr = sched_confs_.find(sname);
    if (itr != sched_confs_.end()) {
      proc_num_ = itr->second.proc_num();
      cpu_binding_start_index_ = itr->second.cpu_binding_start_index();
    }
  }

  // default val for case w/o config:
  if (proc_num_ == 0) {
    proc_num_ = std::thread::hardware_concurrency();
    // FIXME: other vals ... ?
  }
  // Currently for compatible with task/task_manager.cc,
  // will be deleted at last:
  // auto pool_size = scheduler::Scheduler::Instance()->TaskPoolSize();
  task_pool_size_ = proc_num_;

  CreateProcessor();
}

void SchedulerClassic::CreateProcessor() {
  for (uint32_t i = 0; i < proc_num_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ClassicContext>();

    proc->BindContext(ctx);
    proc->SetBindCpuIndex(cpu_binding_start_index_ + i);
    ctx->BindProc(proc);
    pctxs_.emplace_back(ctx);

    proc->Start();
  }
}

bool SchedulerClassic::DispatchTask(const std::shared_ptr<CRoutine> cr) {
  // Check if task exists already.
  for (int i = MAX_PRIO - 1; i >= 0; --i) {
    ReadLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[i]);

    for (auto it = ClassicContext::rq_[i].begin();
          it != ClassicContext::rq_[i].end(); ++it) {
      if ((*it)->id() == cr->id()) {
        return false;
      }
    }
  }

  // Check if task prio is resonable.
  if (cr->priority() < 0 || cr->priority() >= MAX_PRIO) {
    return false;
  }

  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::RT_CREATE, cr->id(),
                                            cr->processor_id());

  // Enqueue task.
  WriteLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[cr->priority()]);
  ClassicContext::rq_[cr->priority()].emplace_back(cr);

  return true;
}

bool SchedulerClassic::NotifyProcessor(uint64_t crid) {
  if (unlikely(stop_)) {
    return true;
  }

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

bool SchedulerClassic::RemoveTask(const std::string& name) {
  if (unlikely(stop_)) {
    return true;
  }

  auto crid = GlobalData::GenerateHashId(name);

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

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
