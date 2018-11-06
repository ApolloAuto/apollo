/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Vesched_infoon 2.0 (the "License");
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
#include "cyber/scheduler/scheduler.h"

#include <utility>

#include "cyber/common/global_data.h"
#include "cyber/common/util.h"
#include "cyber/data/data_visitor.h"
#include "cyber/scheduler/policy/classic.h"
#include "cyber/scheduler/policy/task_choreo.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/processor_context.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::common::GlobalData;

Scheduler::Scheduler() : stop_(false) {
  auto gconf = GlobalData::Instance()->Config();

  for (auto& conf : gconf.scheduler_conf().confs()) {
    sched_confs_[conf.process_name()] = conf;
  }

  SchedConf sconf;
  auto itr =
      sched_confs_.find(GlobalData::Instance()->ProcessName());
  // default conf defined in proto will be used
  // if no specialized conf defined
  if (itr != sched_confs_.end()) {
    sconf = itr->second;
  }
  sched_policy_ = sconf.policy();
  proc_num_ = sconf.proc_num();
  task_pool_size_ = sconf.task_pool_size();
  cpu_binding_start_index_ =
      sconf.cpu_binding_start_index();

  for (auto& conf : gconf.choreo_conf()) {
    if (conf.process_name() ==
        GlobalData::Instance()->ProcessName()) {
      for (auto& choreo : conf.choreos()) {
        cr_confs_[choreo.name()] = choreo;
      }
    }
  }
  CreateProcessor();
}

void Scheduler::CreateProcessor() {
  for (uint32_t i = 0; i < proc_num_; i++) {
    auto proc = std::make_shared<Processor>();
    proc->SetId(i);

    std::shared_ptr<ProcessorContext> ctx;
    switch (sched_policy_) {
      case SchedPolicy::CLASSIC:
        ctx.reset(new ClassicContext());
        break;
      case SchedPolicy::CHOREO:
        ctx.reset(new TaskChoreoContext());
        break;
      default:
        ctx.reset(new TaskChoreoContext());
        break;
    }
    ctx->SetId(i);
    proc->BindContext(ctx);
    ctx->BindProc(proc);
    proc_ctxs_.emplace_back(ctx);
    proc->SetCpuBindingStartIndex(
        cpu_binding_start_index_);
    proc->Start();
  }

  // For taskchoreo policy: put tasks w/o processor assigned to a classic pool.
  if (sched_policy_ == SchedPolicy::CHOREO) {
    for (uint32_t i = 0; i < task_pool_size_; i++) {
      auto proc = std::make_shared<Processor>();

      std::shared_ptr<ProcessorContext> ctx;
      ctx.reset(classic_4_choreo_ = new ClassicContext());
      ctx->SetId(proc_num_ + i);

      proc->BindContext(ctx);
      ctx->BindProc(proc);
      proc_ctxs_.emplace_back(ctx);

      proc->SetCpuBindingStartIndex(
          cpu_binding_start_index_);
      proc->Start();
    }
  }
}

void Scheduler::ShutDown() {
  if (stop_.exchange(true)) {
    return;
  }

  for (auto& proc_ctx : proc_ctxs_) {
    proc_ctx->ShutDown();
  }
  proc_ctxs_.clear();
}

Scheduler::~Scheduler() {}

bool Scheduler::CreateTask(const RoutineFactory& factory,
                           const std::string& name) {
  return CreateTask(factory.create_routine(), name, factory.GetDataVisitor());
}

bool Scheduler::CreateTask(std::function<void()>&& func,
                           const std::string& name,
                           std::shared_ptr<DataVisitorBase> visitor) {
  if (stop_) {
    AERROR << "scheduler is stoped, cannot create task!";
    return false;
  }

  auto task_id = GlobalData::RegisterTaskName(name);
  {
    ReadLockGuard<AtomicRWLock> rg(rw_lock_);
    if (cr_ctx_.find(task_id) != cr_ctx_.end()) {
      AERROR << "Routine [" << name << "] has been exists";
      return false;
    }
    cr_ctx_[task_id] = 0;
  }

  auto cr = std::make_shared<CRoutine>(func);
  cr->set_id(task_id);
  cr->set_name(name);

  Choreo conf;
  if (cr_confs_.find(name) != cr_confs_.end()) {
    conf = cr_confs_[name];
    cr->set_priority(conf.priority());

    if (conf.has_processor_index()) {
      auto proc_id = conf.processor_index();
      cr->set_processor_id(proc_id);
    }
  }

  WriteLockGuard<AtomicRWLock> rg(rw_lock_);
  if (!proc_ctxs_[0]->DispatchTask(cr)) {
    return false;
  }
  if (visitor != nullptr) {
    visitor->RegisterNotifyCallback([this, task_id, name]() {
      if (stop_) {
        return;
      }
      this->NotifyProcessor(task_id);
    });
  }
  return true;
}

bool Scheduler::NotifyTask(uint64_t task_id) {
  if (stop_) {
    return true;
  }
  return NotifyProcessor(task_id);
}

bool Scheduler::NotifyProcessor(uint64_t cr_id) {
  if (stop_) {
    return true;
  }

  ReadLockGuard<AtomicRWLock> rg(rw_lock_);
  auto itr = cr_ctx_.find(cr_id);
  if (itr != cr_ctx_.end()) {
    proc_ctxs_[itr->second]->Notify(cr_id);
    return true;
  }
  return false;
}

bool Scheduler::RemoveTask(const std::string& name) {
  if (stop_) {
    return true;
  }

  auto task_id = GlobalData::RegisterTaskName(name);
  {
    WriteLockGuard<AtomicRWLock> wg(rw_lock_);
    cr_ctx_.erase(task_id);
  }
  return RemoveCRoutine(task_id);
}

bool Scheduler::RemoveCRoutine(uint64_t cr_id) {
  if (stop_) {
    return true;
  }

  WriteLockGuard<AtomicRWLock> rw(rw_lock_);
  auto p = cr_ctx_.find(cr_id);
  if (p != cr_ctx_.end()) {
    cr_ctx_.erase(cr_id);
    proc_ctxs_[p->second]->RemoveCRoutine(cr_id);
  }
  return true;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
