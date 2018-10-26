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
  proc_num_ = std::thread::hardware_concurrency();

  if (gconf.has_scheduler_conf()) {
    sched_conf_.CopyFrom(gconf.scheduler_conf());

    if (sched_conf_.has_processor_conf()) {
      sched_policy_ = sched_conf_.processor_conf().process_strategy();
      proc_num_ = sched_conf_.processor_conf().processor_num();
    } else {
      AERROR << "No processor conf";
      return;
    }

    if (sched_conf_.has_task_pool_conf()) {
      task_pool_size_ = sched_conf_.task_pool_conf().task_pool_size();
    } else {
      AERROR << "No processor conf";
      return;
    }

  } else {
    AERROR << "No scheduler conf";
    return;
  }

  for (auto& conf : gconf.croutine_conf()) {
    if (cr_confs_.find(conf.name()) ==
        cr_confs_.end()) {
      cr_confs_[conf.name()] = conf;
    }
  }

  CreateProcessor();
  //  StartSysmon();
}

void Scheduler::StartSysmon() {
  int interval = 1000;
  struct sched_param param;
  int policy;

  if (sched_conf_.has_sysmon_hz()) {
    interval = (1.0 / sched_conf_.sysmon_hz()) * 1000000;
  }
  sysmon_ = std::thread([=]() {
    while (!stop_) {
      usleep(interval);
    }
  });

  pthread_getschedparam(sysmon_.native_handle(), &policy, &param);
  param.sched_priority = 60;
  if (sched_conf_.has_sysmon_prio()) {
    param.sched_priority = sched_conf_.sysmon_prio();
  }
  pthread_setschedparam(sysmon_.native_handle(), SCHED_FIFO, &param);
}

void Scheduler::CreateProcessor() {
  for (uint32_t i = 0; i < proc_num_ + task_pool_size_; i++) {
    auto proc = std::make_shared<Processor>();
    proc->set_id(i);

    std::shared_ptr<ProcessorContext> ctx;
    switch (sched_policy_) {
      case ProcessStrategy::CLASSIC:
        ctx.reset(new ClassicContext());
        break;
      case ProcessStrategy::CHOREO:
        ctx.reset(new TaskChoreoContext());
        break;
      default:
        ctx.reset(new TaskChoreoContext());
        break;
    }

    ctx->set_id(i);
    proc->bind_context(ctx);
    proc->set_strategy(sched_policy_);
    ctx->bind_processor(proc);
    proc_ctxs_.emplace_back(ctx);
    proc->Start();
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

  if (sysmon_.joinable()) {
    sysmon_.join();
  }
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
    ReadLockGuard<AtomicRWLock> rw(rw_lock_);
    if (cr_ctx_.find(task_id) != cr_ctx_.end()) {
      AERROR << "Routine [" << name << "] has been exists";
      return false;
    }
    cr_ctx_[task_id] = 0;
  }

  if (visitor != nullptr) {
    visitor->RegisterNotifyCallback([this, task_id, name]() {
      if (stop_) {
        return;
      }
      this->NotifyProcessor(task_id);
    });
  }

  auto cr = std::make_shared<CRoutine>(func);
  cr->set_id(task_id);
  cr->set_name(name);

  CRoutineConf conf;
  if (cr_confs_.find(name) != cr_confs_.end()) {
    conf = cr_confs_[name];
    cr->set_priority(conf.priority());
    cr->set_frequency(conf.frequency());

    if (conf.has_processor_index()) {
      auto proc_id = conf.processor_index();
      cr->set_processor_id(proc_id);
    }
  }

  WriteLockGuard<AtomicRWLock> rw(rw_lock_);
  if (!proc_ctxs_[0]->DispatchTask(cr)) {
    return false;
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

  ReadLockGuard<AtomicRWLock> rw(rw_lock_);
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
    WriteLockGuard<AtomicRWLock> rw(rw_lock_);
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
