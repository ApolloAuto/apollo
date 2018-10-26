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

#include "cyber/scheduler/processor.h"

#include "cyber/common/log.h"
#include "cyber/common/types.h"
#include "cyber/croutine/croutine.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/processor_context.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

void ProcessorContext::RemoveCRoutine(uint64_t cr_id) {
  WriteLockGuard<AtomicRWLock> rw(rw_lock_);
  auto it = cr_container_.find(cr_id);
  if (it != cr_container_.end()) {
    it->second->Stop();
    cr_container_.erase(it);
  }
}

int ProcessorContext::RqSize() {
  ReadLockGuard<AtomicRWLock> rw(rw_lock_);
  return cr_container_.size();
}

void ProcessorContext::Notify(uint64_t cr_id) {
  PerfEventCache::Instance()->AddSchedEvent(
      SchedPerf::NOTIFY_IN, cr_id,
      proc_index_);

  ReadLockGuard<AtomicRWLock> rw(rw_lock_);
  if (cr_container_.find(cr_id) ==
      cr_container_.end()) {
    return;
  }
  auto& cr = cr_container_[cr_id];
  if (cr->state() == RoutineState::DATA_WAIT) {
    cr->set_state(RoutineState::READY);
  }

  if (!notified_.exchange(true)) {
    processor_->Notify();
    return;
  }
}

void ProcessorContext::ShutDown() {
  if (!stop_) {
    stop_ = true;
  }
  processor_->Stop();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
