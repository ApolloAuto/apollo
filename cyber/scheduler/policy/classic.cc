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

#include "cyber/scheduler/policy/classic.h"

#include <functional>
#include <memory>

#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

std::mutex ClassicContext::mtx_taskq_;
std::mutex ClassicContext::mtx_rq_;
std::unordered_map<uint64_t, std::shared_ptr<CRoutine>>
    ClassicContext::taskq_;
std::multimap<uint32_t, std::shared_ptr<CRoutine>, std::greater<uint32_t>>
    ClassicContext::rq_;

bool ClassicContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  {
    std::lock_guard<std::mutex> lk(mtx_taskq_);
    if (taskq_.find(cr->id()) != taskq_.end()) {
      return false;
    }
    taskq_[cr->id()] = cr;
  }
  std::lock_guard<std::mutex> lg(mtx_rq_);
  rq_.insert({cr->priority(), cr});
  return true;
}

void ClassicContext::Notify(uint64_t routine_id) {
  ReadLockGuard<AtomicRWLock> lg(rw_lock_);
  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NOTIFY_IN, routine_id,
      proc_index_, 0, 0, -1, -1);
  auto routine = taskq_[routine_id];
  if (routine->state() == RoutineState::DATA_WAIT) {
    auto lock = routine->GetLock();
    if (routine->state() == RoutineState::DATA_WAIT) {
      routine->set_state(RoutineState::READY);
    }
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
  auto start_perf_time = apollo::cyber::Time::Now().ToNanosecond();
  for (auto it = rq_.begin(); it != rq_.end();) {
    auto cr = it->second;
    auto lock = cr->TryLock();
    if (!lock) {
      ++it;
      continue;
    }
    cr->UpdateState();
    if (cr->state() == RoutineState::FINISHED) {
      it = rq_.erase(it);
      continue;
    }

    if (cr->state() == RoutineState::READY) {
      cr->set_state(RoutineState::RUNNING);
      croutine = cr;
      break;
    }
    ++it;
  }

  if (croutine == nullptr) {
    notified_.store(false);
  } else {
    PerfEventCache::Instance()->AddSchedEvent(
        SchedPerf::NEXT_ROUTINE, croutine->id(), croutine->processor_id(), 0,
        start_perf_time, -1, -1);
  }
  return croutine;
}

bool ClassicContext::RqEmpty() {
  std::lock_guard<std::mutex> lg(mtx_rq_);
  return rq_.empty();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
