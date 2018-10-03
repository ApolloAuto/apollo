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

#include "cybertron/scheduler/processor.h"

#include "cybertron/common/log.h"
#include "cybertron/common/types.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/event/perf_event_cache.h"
#include "cybertron/scheduler/processor_context.h"
#include "cybertron/scheduler/scheduler.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::event::PerfEventCache;
using apollo::cybertron::event::SchedPerf;

void ProcessorContext::RemoveCRoutine(uint64_t croutine_id) {
  WriteLockGuard<AtomicRWLock> lg(rw_lock_);
  auto it = cr_map_.find(croutine_id);
  if (it != cr_map_.end()) {
    it->second->Stop();
    cr_map_.erase(it);
  }
}

int ProcessorContext::RqSize() {
  ReadLockGuard<AtomicRWLock> lg(rw_lock_);
  return cr_map_.size();
}

void ProcessorContext::Notify(uint64_t routine_id) {
  PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NOTIFY_IN, routine_id,
                                            proc_index_, 0, 0, -1, -1);
  ReadLockGuard<AtomicRWLock> lg(rw_lock_);
  if (cr_map_[routine_id]->state() != RoutineState::RUNNING) {
    cr_map_[routine_id]->set_state(RoutineState::READY);
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
}  // namespace cybertron
}  // namespace apollo
