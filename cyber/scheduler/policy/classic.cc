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

std::array<AtomicRWLock, MAX_SCHED_PRIORITY>
     ClassicContext::rw_locks_;
std::array<std::vector<std::shared_ptr<CRoutine>>,
    MAX_SCHED_PRIORITY> ClassicContext::rq_;

bool ClassicContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  if (cr->processor_id() != id()) {
    return false;
  }

  {
    WriteLockGuard<AtomicRWLock> rw(rw_lock_);
    if (cr->priority() < 0 || cr->priority() >= MAX_SCHED_PRIORITY) {
      return false;
    }

    if (cr_container_.find(cr->id()) != cr_container_.end()) {
      return false;
    }

    cr_container_[cr->id()] = cr;
  }

  WriteLockGuard<AtomicRWLock> rw_lock(rw_locks_[cr->priority()]);
  rq_[cr->priority()].emplace_back(cr);

  return true;
}

std::shared_ptr<CRoutine> ClassicContext::NextRoutine() {
  if (stop_) {
    return nullptr;
  }

  for (int i = MAX_SCHED_PRIORITY - 1; i >= 0; --i) {
    ReadLockGuard<AtomicRWLock> rw_lock(rw_locks_[i]);
    for (auto it = rq_[i].begin(); it != rq_[i].end();) {
      auto cr = (*it);
      auto lock = cr->TryLock();
      if (!lock) {
        ++it;
        continue;
      }

      cr->UpdateState();
      if (cr->state() == RoutineState::READY) {
        cr->set_state(RoutineState::RUNNING);
        PerfEventCache::Instance()->AddSchedEvent(
            SchedPerf::NEXT_ROUTINE, cr->id(),
            cr->processor_id());
        return cr;
      }
      ++it;
    }
  }

  notified_.store(false);
  return nullptr;
}

bool ClassicContext::RqEmpty() {
  return false;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
