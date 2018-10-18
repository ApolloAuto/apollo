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

#include "cybertron/scheduler/policy/classic.h"
#include "cybertron/scheduler/scheduler.h"
#include "cybertron/scheduler/processor.h"
#include "cybertron/event/perf_event_cache.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::event::PerfEventCache;
using apollo::cybertron::event::SchedPerf;

std::mutex ClassicContext::mtx_taskq_;
std::mutex ClassicContext::mtx_rq_;
std::unordered_multimap<uint64_t, std::shared_ptr<CRoutine>> ClassicContext::taskq_;
std::multimap<uint32_t, std::shared_ptr<CRoutine>, std::greater<uint32_t>> ClassicContext::rq_;

bool ClassicContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  std::lock_guard<std::mutex> lk(mtx_taskq_);
  if (taskq_.find(cr->id()) == taskq_.end()) {
    taskq_.insert({cr->id(), cr});
  }
  return true;
}

void ClassicContext::Notify(uint64_t tid) {
  std::lock_guard<std::mutex> lk(mtx_rq_);
  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NOTIFY_IN, tid, proc_index_, 0, 0, -1, -1);
  auto p = taskq_.find(tid);
  if ( p != taskq_.end() && p->second->state() != RoutineState::RUNNING) {
    p->second->set_state(RoutineState::READY);
    rq_.insert({p->second->priority(), p->second});
  }

  if (!notified_.exchange(true)) {
    processor_->Notify();
    return;
  } 
}

std::shared_ptr<CRoutine> ClassicContext::NextRoutine() {
  if (stop_) {
    return nullptr;
  }
  std::lock_guard<std::mutex> lk(mtx_rq_);
  std::shared_ptr<CRoutine> croutine = nullptr;
  auto start_perf_time = apollo::cybertron::Time::Now().ToNanosecond();
  auto p = rq_.begin();
  for (; p != rq_.end(); ) {
    auto cr = p->second;
    auto lock = cr->TryLock();
    if (!lock) {
      ++p;
      continue;
    }
    if (cr->state() == RoutineState::READY) {
      croutine = cr;
      croutine->set_state(RoutineState::RUNNING);
      rq_.erase(p);
      break;
    }
    ++p;
  }
  if (croutine == nullptr) {
    notified_.store(false);
  } else {
    PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NEXT_ROUTINE, croutine->id(), croutine->processor_id(), 
      0, start_perf_time, -1, -1);
  }
  return croutine;
}

bool ClassicContext::RqEmpty() {
  return rq_.empty();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo