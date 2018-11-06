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
#include <unordered_map>

#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

std::array<AtomicRWLock, MAX_SCHED_PRIORITY> ClassicContext::rw_locks_;
std::array<std::vector<std::shared_ptr<CRoutine>>, MAX_SCHED_PRIORITY>
    ClassicContext::rq_;

bool ClassicContext::DispatchTask(const std::shared_ptr<CRoutine> cr) {
  std::unordered_map<uint64_t, uint32_t>& rt_ctx =
      Scheduler::Instance()->RtCtx();
  if (rt_ctx.find(cr->id()) != rt_ctx.end()) {
    rt_ctx[cr->id()] = proc_index_;
  }
  cr->set_processor_id(proc_index_);

  return Enqueue(cr);
}

bool ClassicContext::Enqueue(const std::shared_ptr<CRoutine> cr) {
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

  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::RT_CREATE, cr->id(),
                                            cr->processor_id());

  WriteLockGuard<AtomicRWLock> rw_lock(rw_locks_[cr->priority()]);
  rq_[cr->priority()].emplace_back(cr);

  return true;
}

std::shared_ptr<CRoutine> ClassicContext::NextRoutine() {
  if (unlikely(stop_)) {
    return nullptr;
  }

  for (int i = MAX_SCHED_PRIORITY - 1; i >= 0; --i) {
    ReadLockGuard<AtomicRWLock> rw_lock(rw_locks_[i]);
    for (auto it = rq_[i].begin(); it != rq_[i].end(); ++it) {
      auto cr = (*it);
      if (!cr->Acquire()) {
        continue;
      }

      if (cr->UpdateState() == RoutineState::READY) {
        PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NEXT_RT, cr->id(),
                                                  cr->processor_id());
        return cr;
      }
      cr->Release();
    }
  }

  notified_.clear();
  return nullptr;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
