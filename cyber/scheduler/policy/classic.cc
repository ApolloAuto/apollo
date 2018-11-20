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

#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/policy/scheduler_classic.h"
#include "cyber/scheduler/processor.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

std::mutex ClassicContext::mtx_wq_;
std::condition_variable ClassicContext::cv_wq_;

std::array<AtomicRWLock, MAX_PRIO> ClassicContext::rq_locks_;
std::array<std::vector<std::shared_ptr<CRoutine>>, MAX_PRIO>
    ClassicContext::rq_;

std::shared_ptr<CRoutine> ClassicContext::NextRoutine() {
  if (unlikely(stop_)) {
    return nullptr;
  }
  for (int i = MAX_PRIO - 1; i >= 0; --i) {
    ReadLockGuard<AtomicRWLock> lk(rq_locks_[i]);
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

  return nullptr;
}

void ClassicContext::Wait() {
  std::unique_lock<std::mutex> lk(mtx_wq_);
  cv_wq_.wait_for(lk, std::chrono::milliseconds(1));
}

void ClassicContext::Notify() {
  cv_wq_.notify_one();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
