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

#include "cybertron/scheduler/policy/classic.h"

#include <utility>

#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/types.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/event/perf_event_cache.h"
#include "cybertron/proto/routine_conf.pb.h"
#include "cybertron/proto/scheduler_conf.pb.h"
#include "cybertron/scheduler/processor.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using croutine::RoutineState;
using apollo::cybertron::common::GlobalData;
using apollo::cybertron::event::PerfEventCache;
using apollo::cybertron::event::SchedPerf;
using apollo::cybertron::proto::SchedulerConf;
using apollo::cybertron::proto::RoutineConf;
using apollo::cybertron::proto::RoutineConfInfo;

std::shared_ptr<CRoutine> ClassicContext::NextRoutine() {
  if (stop_) {
    return nullptr;
  }
  std::lock_guard<std::mutex> lock(mtx_run_queue_);
  auto start_perf_time = apollo::cybertron::Time::Now().ToNanosecond();

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
      cr->set_state(RoutineState::RUNNING);
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

bool ClassicContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  WriteLockGuard<AtomicRWLock> lg(rw_lock_);
  if (cr_map_.find(cr->id()) != cr_map_.end()) {
    return false;
  }
  cr_map_[cr->id()] = cr;
  return true;
}

bool ClassicContext::EnqueueAffinityRoutine(
    const std::shared_ptr<CRoutine>& cr) {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  rt_queue_.insert(
      std::pair<double, std::shared_ptr<CRoutine>>(cr->priority(), cr));
  return true;
}

bool ClassicContext::RqEmpty() {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  return !notified_.load();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
