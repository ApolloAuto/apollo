/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "cybertron/scheduler/processor.h"

#include "cybertron/common/log.h"
#include "cybertron/common/types.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/event/perf_event_cache.h"
#include "cybertron/scheduler/policy/processor_context.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::event::PerfEventCache;

bool ProcessorContext::Pop(uint64_t croutine_id,
                           std::future<std::shared_ptr<CRoutine>>& fut) {
  std::promise<std::shared_ptr<CRoutine>> prom;
  fut = prom.get_future();
  WriteLockGuard lg(rw_lock_);
  if (cr_map_.erase(croutine_id) != 0) {
    pop_list_.Set(croutine_id, std::move(prom));
    return true;
  }
  return false;
}

void ProcessorContext::Push(const std::shared_ptr<CRoutine>& cr) {
  {
    WriteLockGuard lg(rw_lock_);
    if (cr_map_.find(cr->Id()) != cr_map_.end() || pop_list_.Has(cr->Id())) {
      return;
    }
    cr_map_[cr->Id()] = cr;
  }
  Enqueue(cr);
}

void ProcessorContext::RemoveCRoutine(uint64_t croutine_id) {
  WriteLockGuard lg(rw_lock_);
  auto it = cr_map_.find(croutine_id);
  if (it != cr_map_.end()) {
    it->second->Stop();
    cr_map_.erase(it);
  }
}

void ProcessorContext::NotifyProcessor(uint64_t routine_id) {
  ReadLockGuard lg(rw_lock_);
  if (cr_map_.find(routine_id) == cr_map_.end()) {
    return;
  }

  if (!cr_map_[routine_id]->IsRunning()) {
    cr_map_[routine_id]->SetState(RoutineState::READY);
  }
  if (!notified_.exchange(true)) {
    PerfEventCache::Instance()->AddSchedEvent(4, routine_id, proc_index_, 0, 0);
    processor_->Notify();
    return;
  }

  if (processor_->EnableEmergencyThread()) {
    if (!cr_map_[routine_id]->IsRunning() && IsPriorInverse(routine_id)) {
      processor_->NotifyEmergencyThread();
    }
  }
}

void ProcessorContext::ShutDown() {
  if (!stop_) {
    stop_ = true;
  }
  processor_->Stop();
}

// TODO  use perf event
void ProcessorContext::PrintStatistics() {
  double average_proc_time = 0.0;
  uint64_t proc_interval_nano = proc_interval_.count();
  if (proc_num_ > 0) {
    average_proc_time = static_cast<double>(proc_interval_nano) / proc_num_;
  }
  AINFO << "strategy_proc_time(us)["
        << static_cast<double>(proc_interval_nano) / 1000 << "] pop_num["
        << proc_num_ << "] average_proc_time(us)[" << average_proc_time / 1000
        << "]";
}

void ProcessorContext::PrintCRoutineStats() {
  ReadLockGuard lg(rw_lock_);
  for (auto it = cr_map_.begin(); it != cr_map_.end(); ++it) {
    it->second->PrintStatistics();
  }
}

void ProcessorContext::UpdateProcessStat(ProcessorStat* stat) {
  ReadLockGuard lg(rw_lock_);
  for (auto it = cr_map_.begin(); it != cr_map_.end(); ++it) {
    auto s = it->second->GetStatistics();
    stat->exec_time += s.exec_time;
    stat->sleep_time += s.sleep_time;
  }
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
