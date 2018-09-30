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

#include "cybertron/scheduler/scheduler.h"
#include "cybertron/common/global_data.h"
#include "cybertron/common/util.h"
#include "cybertron/data/data_visitor.h"
#include "cybertron/scheduler/policy/cfs.h"
#include "cybertron/scheduler/policy/classic.h"
#include "cybertron/scheduler/processor.h"
#include "cybertron/scheduler/processor_context.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::proto::RoutineConfInfo;
using apollo::cybertron::common::GlobalData;

Scheduler::Scheduler() : stop_(false) {
  auto gconf = GlobalData::Instance()->Config();
  int interval = 1000;
  struct sched_param param;
  int policy;
  proc_num_ = std::thread::hardware_concurrency();

  if (gconf.has_scheduler_conf()) {
    sched_conf_.CopyFrom(gconf.scheduler_conf());

    if (sched_conf_.has_processor_conf()) {
      sched_policy_ = sched_conf_.processor_conf().process_strategy();

      if (sched_conf_.processor_conf().processor_num() > 0) {
        proc_num_ = sched_conf_.processor_conf().processor_num();
      }
    } else {
      AERROR << "No processor conf";
      return;
    }
  } else {
    AERROR << "No scheduler conf";
    return;
  }

  if (gconf.has_routine_conf()) {
    rt_conf_.CopyFrom(gconf.routine_conf());
  } else {
    AWARN << "No routine conf";
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
  for (int i = 0; i < proc_num_; i++) {
    auto proc = std::make_shared<Processor>();
    proc->SetId(i);

    std::shared_ptr<ProcessorContext> ctx;
    switch (sched_policy_) {
      case ProcessStrategy::CLASSIC:
        classic_ = true;
        ctx.reset(new ClassicContext());
        break;
      case ProcessStrategy::CFS:
        ctx.reset(new CFSContext());
        break;
      default:
        ctx.reset(new CFSContext());
        break;
    }

    ctx->SetId(i);
    proc->BindContext(ctx);
    ctx->BindProcessor(proc);
    proc_ctxs_.emplace_back(ctx);
    {
      std::lock_guard<std::mutex> lk(mtx_ctx_qsize_);
      ctx_qsize_.insert(std::pair<int, std::shared_ptr<ProcessorContext>>(
          ctx->RqSize(), ctx));
    }
    proc->Start();
  }
}

void Scheduler::ShutDown() {
  if (stop_.exchange(true)) {
    return;
  }

  if (sysmon_.joinable()) {
    sysmon_.join();
  }
}

Scheduler::~Scheduler() {}

std::shared_ptr<ProcessorContext> Scheduler::FindProc(
    const std::shared_ptr<CRoutine>& cr) {
  int cur_proc_ = -1;
  int count = 0;
  auto cur_cr = CRoutine::GetCurrentRoutine();

  if (cur_cr) {
    cur_proc_ = cur_cr->ProcessorId();
  }

  if (cr->ProcessorId() != -1 && cr->ProcessorId() != cur_proc_ &&
      cr->ProcessorId() < proc_ctxs_.size()) {
    cr->SetProcessorId(proc_ctxs_[cr->ProcessorId()]->Id());
    return proc_ctxs_[cr->ProcessorId()];
  }

  {
    std::lock_guard<std::mutex> lk(mtx_ctx_qsize_);
    for (auto p : ctx_qsize_) {
      auto ctx = p.second;

      int proc_id = ctx->Id();
      if (proc_id == cur_proc_ || !cr->IsAffinity(proc_id)) {
        continue;
      }
      cr->SetProcessorId(ctx->Id());
      return ctx;
    }
  }

  for (int i = 0; i < proc_ctxs_.size(); i++) {
    count += proc_ctxs_[i]->RqSize();
  }
  auto id = count % proc_ctxs_.size();
  cr->SetProcessorId(proc_ctxs_[id]->Id());
  return proc_ctxs_[cr->ProcessorId()];
}

bool Scheduler::DispatchTask(const std::shared_ptr<CRoutine>& croutine) {
  auto ctx = FindProc(croutine);

  if (croutine->ProcessorId() >= proc_num_ ||
      croutine->ProcessorId() < 0 && ctx) {
    AERROR << GlobalData::GetTaskNameById(croutine->Id())
           << "push failed, get processor failed, "
           << "target processor index: " << croutine->ProcessorId();
    return false;
  }

  if (ctx->Enqueue(croutine)) {
    ADEBUG << "push routine[" << GlobalData::GetTaskNameById(croutine->Id())
           << "] into processor[" << croutine->ProcessorId() << "]";
  } else {
    AWARN << "push routine[" << GlobalData::GetTaskNameById(croutine->Id())
          << "] into processor[" << croutine->ProcessorId() << "] failed";
    return false;
  }

  for (int i = 0; i < proc_ctxs_.size(); ++i) {
    if (croutine->IsAffinity(i)) {
      if (proc_ctxs_.at(i)->EnqueueAffinityRoutine(croutine)) {
        ADEBUG << "push routine[" << GlobalData::GetTaskNameById(croutine->Id())
               << "] into affinity_processor[" << i << "]";
      } else {
        AWARN << "push routine[" << GlobalData::GetTaskNameById(croutine->Id())
              << "] into affinity_processor[" << i << "] failed";
      }
    }
  }

  rt_ctx_.insert({croutine->Id(), croutine->ProcessorId()});

  {
    std::lock_guard<std::mutex> lk(mtx_ctx_qsize_);
    std::shared_ptr<ProcessorContext> pctx = nullptr;
    for (auto it = ctx_qsize_.begin(); it != ctx_qsize_.end(); ++it) {
      if (it->second->Id() == croutine->ProcessorId()) {
        pctx = it->second;
        ctx_qsize_.erase(it);
        break;
      }
    }
    if (pctx) {
      ctx_qsize_.insert(std::pair<int, std::shared_ptr<ProcessorContext>>(
          pctx->RqSize(), pctx));
    }
  }

  return true;
}

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
    std::lock_guard<std::mutex> lock(task_id_map_mutex_);
    if (task_id_map_.find(task_id) != task_id_map_.end()) {
      AERROR << "Routine [" << name << "] has been exists";
      return false;
    }
    task_id_map_[task_id] = name;
  }

  if (visitor != nullptr) {
    visitor->RegisterNotifyCallback([this, task_id, name]() {
      if (stop_) {
        return;
      }
      this->NotifyProcessor(task_id);
    });
  }

  RoutineConfInfo cr_info;
  for (const auto& routine_info : rt_conf_.routine_info()) {
    if (routine_info.routine_name() == name) {
      cr_info.CopyFrom(routine_info);
      break;
    }
  }

  auto croutine = std::make_shared<CRoutine>(func);
  croutine->SetId(task_id);
  croutine->SetName(name);
  croutine->SetPriority(cr_info.priority());
  croutine->SetFrequency(cr_info.frequency());

  if (cr_info.has_processor_index()) {
    auto processor_id = cr_info.processor_index();
    croutine->SetProcessorId(processor_id);
  }

  if (!cr_info.affinity_processor().empty()) {
    for (const auto& p : cr_info.affinity_processor()) {
      croutine->AddAffinityProcessor(p);
    }
  } else {
    for (int i = 0; i < proc_num_; i++) {
      croutine->AddAffinityProcessor(i);
    }
  }

  if (!DispatchTask(croutine)) {
    return false;
  }

  return true;
}

bool Scheduler::NotifyTask(uint64_t task_id) const {
  if (stop_) {
    return true;
  }
  return NotifyProcessor(task_id);
}

bool Scheduler::NotifyProcessor(uint64_t cr_id) const {
  if (stop_) {
    return true;
  }

  auto itr = rt_ctx_.find(cr_id);
  if (itr != rt_ctx_.end()) {
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
    std::lock_guard<std::mutex> lock(task_id_map_mutex_);
    task_id_map_.erase(task_id);
  }

  return RemoveCRoutine(task_id);
}

bool Scheduler::RemoveCRoutine(uint64_t cr_id) {
  {
    std::lock_guard<std::mutex> lock(task_id_map_mutex_);
    task_id_map_.erase(cr_id);
  }
  auto p = rt_ctx_.find(cr_id);

  if (p != rt_ctx_.end()) {
    rt_ctx_.erase(cr_id);
    proc_ctxs_[p->second]->RemoveCRoutine(cr_id);
  }
  return true;
}

void Scheduler::PrintStatistics() {}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
