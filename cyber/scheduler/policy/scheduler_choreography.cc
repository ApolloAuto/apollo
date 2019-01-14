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

#include <sched.h>
#include <memory>
#include <string>
#include <utility>

#include "cyber/common/environment.h"
#include "cyber/common/file.h"
#include "cyber/scheduler/policy/choreography_context.h"
#include "cyber/scheduler/policy/classic_context.h"
#include "cyber/scheduler/processor.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;
using apollo::cyber::common::GlobalData;
using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::PathExists;
using apollo::cyber::common::GetProtoFromFile;
using apollo::cyber::common::WorkRoot;
using apollo::cyber::croutine::RoutineState;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

SchedulerChoreography::SchedulerChoreography() {
  // get sched config
  std::string conf("conf/");
  conf.append(GlobalData::Instance()->ProcessGroup()).append(".conf");
  auto cfg_file = GetAbsolutePath(WorkRoot(), conf);

  apollo::cyber::proto::CyberConfig cfg;
  if (PathExists(cfg_file) && GetProtoFromFile(cfg_file, &cfg)) {
    proc_num_ =
        cfg.scheduler_conf().choreography_conf().choreography_processor_num();
    choreography_affinity_ =
        cfg.scheduler_conf().choreography_conf().choreography_affinity();
    choreography_processor_policy_ = cfg.scheduler_conf()
                                         .choreography_conf()
                                         .choreography_processor_policy();
    choreography_processor_prio_ =
        cfg.scheduler_conf().choreography_conf().choreography_processor_prio();
    ParseCpuset(cfg.scheduler_conf().choreography_conf().choreography_cpuset(),
                &choreography_cpuset_);

    task_pool_size_ =
        cfg.scheduler_conf().choreography_conf().pool_processor_num();
    pool_affinity_ = cfg.scheduler_conf().choreography_conf().pool_affinity();
    pool_processor_policy_ =
        cfg.scheduler_conf().choreography_conf().pool_processor_policy();
    pool_processor_prio_ =
        cfg.scheduler_conf().choreography_conf().pool_processor_prio();
    ParseCpuset(cfg.scheduler_conf().choreography_conf().pool_cpuset(),
                &pool_cpuset_);

    for (auto& thr : cfg.scheduler_conf().choreography_conf().threads()) {
      inner_thr_confs_[thr.name()] = thr;
    }

    for (auto& task : cfg.scheduler_conf().choreography_conf().tasks()) {
      cr_confs_[task.name()] = task;
    }
  }

  // default val for case w/o config:
  if (proc_num_ == 0) {
    auto& global_conf = GlobalData::Instance()->Config();
    if (global_conf.has_scheduler_conf() &&
        global_conf.scheduler_conf().has_default_proc_num()) {
      proc_num_ = global_conf.scheduler_conf().default_proc_num();
    } else {
      proc_num_ = 2;
    }
    task_pool_size_ = proc_num_;
  }

  CreateProcessor();
}

void SchedulerChoreography::CreateProcessor() {
  for (uint32_t i = 0; i < proc_num_; i++) {
    auto proc = std::make_shared<Processor>();
    auto ctx = std::make_shared<ChoreographyContext>();

    proc->BindContext(ctx);
    proc->SetAffinity(choreography_cpuset_, choreography_affinity_, i);
    proc->SetSchedPolicy(choreography_processor_policy_,
                         choreography_processor_prio_);
    pctxs_.emplace_back(ctx);
    processors_.emplace_back(proc);
  }

  // Put tasks w/o processor assigned into a classic pool.
  for (uint32_t i = 0; i < task_pool_size_; i++) {
    auto ctx = std::make_shared<ClassicContext>();

    auto proc = std::make_shared<Processor>();
    proc->BindContext(ctx);
    proc->SetAffinity(pool_cpuset_, pool_affinity_, i);
    proc->SetSchedPolicy(pool_processor_policy_, pool_processor_prio_);
    pctxs_.emplace_back(ctx);
    processors_.emplace_back(proc);
  }
}

bool SchedulerChoreography::DispatchTask(const std::shared_ptr<CRoutine>& cr) {
  // we use multi-key mutex to prevent race condition
  // when del && add cr with same crid
  MutexWrapper *wrapper = nullptr;
  if (!id_map_mutex_.Get(cr->id(), &wrapper)) {
    {
      std::lock_guard<std::mutex> wl_lg(cr_wl_mtx_);
      if (!id_map_mutex_.Get(cr->id(), &wrapper)) {
        wrapper = new MutexWrapper();
        id_map_mutex_.Set(cr->id(), wrapper);
      }
    }
  }
  std::lock_guard<std::mutex> lg(wrapper->Mutex());

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
    static_cast<ChoreographyContext*>(pctxs_[pid].get())->Enqueue(cr);
  } else {
    // fallback for tasks w/o processor assigned.

    // Check if task prio is reasonable.
    if (cr->priority() >= MAX_PRIO) {
      AWARN << cr->name() << " prio great than MAX_PRIO.";
      cr->set_priority(MAX_PRIO - 1);
    }

    cr->set_group_name(DEFAULT_GROUP_NAME);

    // Enqueue task to pool runqueue.
    {
      WriteLockGuard<AtomicRWLock> lk(
          ClassicContext::rq_locks_[DEFAULT_GROUP_NAME].at(cr->priority()));
      ClassicContext::cr_group_[DEFAULT_GROUP_NAME].at(cr->priority())
          .emplace_back(cr);
    }
  }
  return true;
}

bool SchedulerChoreography::RemoveTask(const std::string& name) {
  if (unlikely(stop_)) {
    return true;
  }

  auto crid = GlobalData::GenerateHashId(name);
  return RemoveCRoutine(crid);
}
bool SchedulerChoreography::RemoveCRoutine(uint64_t crid) {
  // we use multi-key mutex to prevent race condition
  // when del && add cr with same crid
  MutexWrapper *wrapper = nullptr;
  if (!id_map_mutex_.Get(crid, &wrapper)) {
    {
      std::lock_guard<std::mutex> wl_lg(cr_wl_mtx_);
      if (!id_map_mutex_.Get(crid, &wrapper)) {
        wrapper = new MutexWrapper();
        id_map_mutex_.Set(crid, wrapper);
      }
    }
  }
  std::lock_guard<std::mutex> lg(wrapper->Mutex());

  // Find cr from id_cr &&
  // get cr prio if cr found
  int prio;
  int pid;
  std::string group_name;
  {
    WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
    auto p = id_cr_.find(crid);
    if (p != id_cr_.end()) {
      auto cr = p->second;
      prio = cr->priority();
      pid = cr->processor_id();
      id_cr_[crid]->Stop();
      id_cr_.erase(crid);
    } else {
      return false;
    }
  }

  // rm cr from pool if rt not in choreo context
  if (pid == -1) {
    WriteLockGuard<AtomicRWLock> lk(
        ClassicContext::rq_locks_[group_name].at(prio));
    for (auto it = ClassicContext::cr_group_[group_name].at(prio).begin();
         it != ClassicContext::cr_group_[group_name].at(prio).end(); ++it) {
      if ((*it)->id() == crid) {
        auto cr = *it;

        cr->Stop();
        ClassicContext::cr_group_[group_name].at(prio).erase(it);
        cr->Release();
        return true;
      }
    }
  } else {
    static_cast<ChoreographyContext *>(pctxs_[pid].get())
        ->RemoveCRoutine(crid);
    return true;
  }

  return false;
}

bool SchedulerChoreography::NotifyProcessor(uint64_t crid) {
  if (unlikely(stop_)) {
    return true;
  }

  std::shared_ptr<CRoutine> cr;
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

  if (cr->processor_id() != -1) {
    // Notify processor in choreo context.
    auto pid = cr->processor_id();
    static_cast<ChoreographyContext*>(pctxs_[pid].get())->Notify();
  } else {
    // Notify processor in pool.
    ClassicContext::Notify(cr->group_name());
  }

  return true;
}

void SchedulerChoreography::SetInnerThreadAttr(const std::thread* thr,
                                               const std::string& name) {
  if (inner_thr_confs_.find(name) != inner_thr_confs_.end()) {
    auto th = const_cast<std::thread*>(thr);
    auto th_conf = inner_thr_confs_[name];
    auto cpuset = th_conf.cpuset();

    std::vector<int> cpus;
    ParseCpuset(cpuset, &cpus);
    cpu_set_t set;
    CPU_ZERO(&set);
    for (const auto cpu : cpus) {
      CPU_SET(cpu, &set);
    }
    pthread_setaffinity_np(th->native_handle(), sizeof(set), &set);

    auto policy = th_conf.policy();
    auto prio = th_conf.prio();
    int p;
    if (!policy.compare("SCHED_FIFO")) {
      p = SCHED_FIFO;
    } else if (!policy.compare("SCHED_RR")) {
      p = SCHED_RR;
    } else {
      return;
    }

    struct sched_param sp;
    memset(static_cast<void*>(&sp), 0, sizeof(sp));
    sp.sched_priority = prio;
    pthread_setschedparam(th->native_handle(), p, &sp);
  }
  return;
}
}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
