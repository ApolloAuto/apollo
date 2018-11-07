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

#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/common/types.h"
#include "cyber/croutine/croutine.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;
using croutine::RoutineState;

std::shared_ptr<CRoutine> TaskChoreoContext::NextRoutine() {
  if (unlikely(stop_)) {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(mtx_);
  for (auto it = cr_queue_.begin(); it != cr_queue_.end();) {
    auto cr = it->second;
    // FIXME: Remove Acquire() and Release() if there is no race condtion.
    if (!cr->Acquire()) {
      continue;
    }

    if (cr->state() == RoutineState::FINISHED) {
      it = cr_queue_.erase(it);
      continue;
    }
    if (cr->UpdateState() == RoutineState::READY) {
      PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NEXT_RT, cr->id(),
                                                cr->processor_id());
      return cr;
    }
    cr->Release();
    ++it;
  }

  notified_.clear();
  return nullptr;
}

bool TaskChoreoContext::DispatchTask(const std::shared_ptr<CRoutine> cr) {
  auto& rt_ctx =
      Scheduler::Instance()->RtCtx();
  if (rt_ctx.find(cr->id()) == rt_ctx.end()) {
    return false;
  }

  uint32_t pnum = Scheduler::Instance()->ProcessorNum();
  uint32_t psize = Scheduler::Instance()->TaskPoolSize();
  uint32_t pid = cr->processor_id();
  if (pid >= 0 && pid < pnum) {
    rt_ctx[cr->id()] = pid;
    auto ctxs =
        Scheduler::Instance()->ProcCtxs();
    return ctxs[pid]->Enqueue(cr);
  } else {
    // fallback for those w/o processor assigned.
    // FIXME: trick for task pool index set
    rt_ctx[cr->id()] = pnum + psize -1;
    return Scheduler::Instance()->Classic4Choreo()->DispatchTask(cr);
  }
}

bool TaskChoreoContext::Enqueue(const std::shared_ptr<CRoutine> cr) {
  {
    WriteLockGuard<AtomicRWLock> lg(rw_lock_);
    if (cr_container_.find(cr->id()) != cr_container_.end()) {
      return false;
    }
    cr_container_[cr->id()] = cr;
  }

  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::RT_CREATE, cr->id(),
                                            cr->processor_id());

  std::lock_guard<std::mutex> lg(mtx_);
  cr_queue_.insert(
    std::pair<uint32_t, std::shared_ptr<CRoutine>>(cr->priority(), cr));
  return true;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
