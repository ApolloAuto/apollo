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

#include "cyber/scheduler/policy/choreography.h"

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

std::shared_ptr<CRoutine> ChoreographyContext::NextRoutine() {
  if (unlikely(stop_)) {
    return nullptr;
  }

  std::lock_guard<std::mutex> lock(mtx_cr_queue_);
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

bool ChoreographyContext::Enqueue(const std::shared_ptr<CRoutine> cr) {
  {
    WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
    if (id_cr_.find(cr->id()) != id_cr_.end()) {
      return false;
    }
    id_cr_[cr->id()] = cr;
  }

  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::RT_CREATE, cr->id(),
                                            cr->processor_id());

  std::lock_guard<std::mutex> lk(mtx_cr_queue_);
  cr_queue_.insert(
    std::pair<uint32_t, std::shared_ptr<CRoutine>>(cr->priority(), cr));
  return true;
}

void ChoreographyContext::Notify(uint64_t crid) {
  ReadLockGuard<AtomicRWLock> lk(id_cr_lock_);

  auto it = id_cr_.find(crid);
  if (it != id_cr_.end()) {
    auto& cr = it->second;
    if (cr->state() == RoutineState::DATA_WAIT) {
      cr->SetUpdateFlag();
    }

    if (!notified_.test_and_set(std::memory_order_acquire)) {
      processor_->Notify();
      return;
    }
  }
}

void ChoreographyContext::RemoveCRoutine(uint64_t crid) {
  WriteLockGuard<AtomicRWLock> lk(id_cr_lock_);
  auto it = id_cr_.find(crid);
  if (it != id_cr_.end()) {
    it->second->Stop();
    id_cr_.erase(it);
  }
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
