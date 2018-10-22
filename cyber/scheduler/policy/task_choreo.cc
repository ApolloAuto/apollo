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

#include "cyber/scheduler/policy/task_choreo.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/common/types.h"
#include "cyber/croutine/croutine.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/processor.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;
using croutine::RoutineState;

std::shared_ptr<CRoutine> TaskChoreoContext::NextRoutine() {
  if (stop_) {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(mtx_run_queue_);
  auto start_perf_time = apollo::cyber::Time::Now().ToNanosecond();

  std::shared_ptr<CRoutine> croutine = nullptr;
  for (auto it = rt_queue_.begin(); it != rt_queue_.end();) {
    auto cr = it->second;
    auto lock = cr->TryLock();
    if (!lock) {
      ++it;
      continue;
    }

    cr->UpdateState();

    if (cr->state() == RoutineState::FINISHED) {
      it = rt_queue_.erase(it);
      continue;
    }

    if (cr->state() == RoutineState::READY) {
      croutine = cr;
      croutine->set_state(RoutineState::RUNNING);
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

bool TaskChoreoContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  if (cr->processor_id() != id()) {
    return false;
  }

  {
    WriteLockGuard<AtomicRWLock> lg(rw_lock_);
    if (cr_container_.find(cr->id()) != cr_container_.end()) {
      return false;
    }
    cr_container_[cr->id()] = cr;
  }

  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  rt_queue_.insert(
      std::pair<uint32_t, std::shared_ptr<CRoutine>>(cr->priority(), cr));
  return true;
}

bool TaskChoreoContext::RqEmpty() {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  return !notified_.load();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
