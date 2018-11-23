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

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/policy/classic.h"
#include "cyber/scheduler/processor.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::croutine::RoutineState;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;
using apollo::cyber::common::GlobalData;
using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::PathExists;
using apollo::cyber::common::GetProtoFromFile;
using apollo::cyber::common::WorkRoot;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

SchedulerClassic::SchedulerClassic() {
  // get sched config
  std::string conf("conf/");
  conf.append(GlobalData::Instance()->ProcessGroup()).append(".conf");
  auto cfg_file = GetAbsolutePath(WorkRoot(), conf);

  apollo::cyber::proto::CyberConfig cfg;
  // FIXME: later, we will add grp support for classic policy.
  if (PathExists(cfg_file) && GetProtoFromFile(cfg_file, &cfg)) {
    auto& groups = cfg.scheduler_conf().classic_conf().groups();
    proc_num_ = groups[0].processor_num();
    affinity_ = groups[0].affinity();
    ParseCpuset(groups[0].cpuset(), &cpuset_);

    for (auto& group : groups) {
      for (auto& task : group.tasks()) {
        cr_tasks[task.name()] = task;
      }
    }
  } else {
    // fallback default sched config. To avoid every process launchs
    // too many threads, limit to 2 cpus for process w/o explicit config.
    proc_num_ = 2;
    // FIXME: other vals ... ?
  }

  // Currently for compatible with task/task_manager.cc:
  // auto pool_size = scheduler::Scheduler::Instance()->TaskPoolSize();
  // which will be deleted at last.
  task_pool_size_ = proc_num_;

  CreateProcessor();
}

void SchedulerClassic::CreateProcessor() {
  for (uint32_t i = 0; i < proc_num_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ClassicContext>();

    proc->BindContext(ctx);
    proc->SetAffinity(cpuset_, affinity_, i);
    ctx->BindProc(proc);
    pctxs_.emplace_back(ctx);
  }
}

bool SchedulerClassic::DispatchTask(const std::shared_ptr<CRoutine> cr) {
  // we use multi-key mutex to prevent race condition
  // when del && add cr with same crid
  std::lock_guard<std::mutex> lg(id_cr_wl_[cr->id()]);

  {
    WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
    if (id_cr_.find(cr->id()) != id_cr_.end()) {
      return false;
    }
    id_cr_[cr->id()] = cr;
  }

  if (cr_tasks.find(cr->name()) != cr_tasks.end()) {
    cr->set_priority(cr_tasks[cr->name()].prio());
  }

  // Check if task prio is reasonable.
  if (cr->priority() >= MAX_PRIO) {
    AWARN << cr->name() << " prio great than MAX_PRIO.";
    cr->set_priority(MAX_PRIO - 1);
  }

  // Enqueue task.
  WriteLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[cr->priority()]);
  ClassicContext::rq_[cr->priority()].emplace_back(cr);

  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::RT_CREATE, cr->id(),
                                            cr->processor_id());
  return true;
}

bool SchedulerClassic::NotifyProcessor(uint64_t crid) {
  if (unlikely(stop_)) {
    return true;
  }

  {
    ReadLockGuard<AtomicRWLock> lk(id_cr_lock_);
    if (id_cr_.find(crid) != id_cr_.end()) {
        auto cr = id_cr_[crid];
        if (cr->state() == RoutineState::DATA_WAIT) {
          cr->SetUpdateFlag();
        }

        ClassicContext::Notify();

        return true;
    }
  }
  return false;
}

bool SchedulerClassic::RemoveTask(const std::string& name) {
  if (unlikely(stop_)) {
    return true;
  }

  auto crid = GlobalData::GenerateHashId(name);

  // we use multi-key mutex to prevent race condition
  // when del && add cr with same crid
  std::lock_guard<std::mutex> lg(id_cr_wl_[crid]);

  std::shared_ptr<CRoutine> cr = nullptr;
  int prio;
  {
    ReadLockGuard<AtomicRWLock> lk(id_cr_lock_);
    if (id_cr_.find(crid) != id_cr_.end()) {
      cr = id_cr_[crid];
      prio = cr->priority();
    } else {
      return false;
    }
  }

  WriteLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[prio]);
  for (auto it = ClassicContext::rq_[prio].begin();
       it != ClassicContext::rq_[prio].end(); ++it) {
    if ((*it)->id() == crid) {
      ClassicContext::rq_[prio].erase(it);
      return true;
    }
  }

  {
    WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
    if (id_cr_.find(crid) != id_cr_.end()) {
      id_cr_[crid]->Stop();
      id_cr_.erase(crid);
    }
  }

  return false;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
