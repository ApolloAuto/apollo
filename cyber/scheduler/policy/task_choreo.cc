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
#include <unordered_map>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/common/types.h"
#include "cyber/croutine/croutine.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/processor.h"
#include "cyber/time/time.h"
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
    auto lock = cr->TryLock();
    if (!lock) {
      ++it;
      continue;
    }

    cr->UpdateState();
    if (cr->state() == RoutineState::FINISHED) {
      it = cr_queue_.erase(it);
      continue;
    }
    if (cr->state() == RoutineState::READY) {
      cr->set_state(RoutineState::RUNNING);
        PerfEventCache::Instance()->AddSchedEvent(
            SchedPerf::NEXT_RT, cr->id(),
            cr->processor_id());
      return cr;
    }
    ++it;
  }

  notified_.store(false);
  return nullptr;
}

bool TaskChoreoContext::DispatchTask(const std::shared_ptr<CRoutine> cr) {
  std::vector<std::shared_ptr<ProcessorContext>> ctxs =
    Scheduler::Instance()->ProcCtxs();

  uint32_t pid = cr->processor_id();
  if (!(pid >= 0 && pid < ctxs.size())) {
    // fallback for those w/o proc idx in conf
    pid = 0;
    int qsize = ctxs[pid]->RqSize();
    for (uint32_t i = 1; i < ctxs.size(); i++) {
      if (qsize > ctxs[i]->RqSize()) {
        qsize = ctxs[i]->RqSize();
        pid = i;
      }
    }
    cr->set_processor_id(pid);
  }

  std::unordered_map<uint64_t, uint32_t>& rt_ctx =
    Scheduler::Instance()->RtCtx();
  if (rt_ctx.find(cr->id()) != rt_ctx.end()) {
    rt_ctx[cr->id()] = cr->processor_id();
  }

  return ctxs[pid]->Enqueue(cr);
}

bool TaskChoreoContext::Enqueue(const std::shared_ptr<CRoutine> cr) {
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

  PerfEventCache::Instance()->AddSchedEvent(
      SchedPerf::RT_CREATE, cr->id(),
      cr->processor_id());

  std::lock_guard<std::mutex> lg(mtx_);
  cr_queue_.insert(
      std::pair<uint32_t, std::shared_ptr<CRoutine>>(cr->priority(), cr));
  return true;
}

bool TaskChoreoContext::RqEmpty() {
  return !notified_.load();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
