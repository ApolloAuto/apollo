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

#include "cybertron/scheduler/policy/cfs.h"

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

std::shared_ptr<CRoutine> CFSContext::NextRoutine() {
  if (stop_) {
    return nullptr;
  }
  std::shared_ptr<CRoutine> cr = NextLocalRoutine();
  if (!cr) {
    cr = NextAffinityRoutine();
  }
  return cr;
}
std::shared_ptr<CRoutine> CFSContext::NextLocalRoutine() {
  std::lock_guard<std::mutex> lock(mtx_run_queue_);
  auto start_perf_time = apollo::cybertron::Time::Now().ToNanosecond();

  // re-enqueue top croutine
  if (cur_croutine_) {
    cur_croutine_->SetVRunningTime(cur_croutine_->ExecTime() /
                                   cur_croutine_->Priority());
    cur_croutine_->SetExecTime(cur_croutine_->VRunningTime() *
                               cur_croutine_->Priority());
    local_rb_map_.insert(std::pair<double, std::shared_ptr<CRoutine>>(
        cur_croutine_->VRunningTime(), cur_croutine_));
    cur_croutine_ = nullptr;
  }

  std::shared_ptr<CRoutine> croutine = nullptr;
  for (auto it = local_rb_map_.begin(); it != local_rb_map_.end();) {
    auto cr = it->second;
    if (!cr->TryLockForOp()) {
      ++it;
      continue;
    }

    auto cr_id = cr->Id();
    cr->UpdateState();
    if (cr->IsFinished()) {
      it = local_rb_map_.erase(it);
      cr->TryUnlockForOp();
      continue;
    }

    if (cr->IsReady()) {
      min_vruntime_ = cr->VRunningTime();
      croutine = cr;
      croutine->SetState(RoutineState::RUNNING);
      cur_croutine_ = croutine;
      local_rb_map_.erase(it);
      cr->TryUnlockForOp();
      break;
    }
    cr->TryUnlockForOp();
    ++it;
  }
  if (croutine == nullptr) {
    notified_.store(false);
  } else {
    PerfEventCache::Instance()->AddSchedEvent(
        SchedPerf::NEXT_ROUTINE, croutine->Id(), croutine->ProcessorId(), 0,
        start_perf_time, -1, -1);
  }
  return croutine;
}

std::shared_ptr<CRoutine> CFSContext::NextAffinityRoutine() {
  if (stop_) {
    return nullptr;
  }
  std::lock_guard<std::mutex> lg(rw_affinity_lock_);
  auto start_perf_time = apollo::cybertron::Time::Now().ToNanosecond();
  std::shared_ptr<CRoutine> croutine = nullptr;

  for (auto it = affinity_rb_map_.begin(); it != affinity_rb_map_.end();) {
    auto cr = it->second;
    if (!cr->TryLockForOp()) {
      ++it;
      continue;
    }

    cr->UpdateState();
    if (cr->IsFinished()) {
      it = affinity_rb_map_.erase(it);
      cr->TryUnlockForOp();
      continue;
    }

    if (cr->IsReady()) {
      croutine = cr;
      cr->SetState(RoutineState::RUNNING);
      cr->TryUnlockForOp();
      break;
    }
    cr->TryUnlockForOp();
    ++it;
  }
  if (croutine) {
    PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NEXT_AFFINITY_R,
                                              croutine->Id(), proc_index_, 0,
                                              start_perf_time, -1, -1);
  }
  return croutine;
}

bool CFSContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  if (cr->ProcessorId() != Id()) {
    return false;
  }

  {
    WriteLockGuard<AtomicRWLock> lg(rw_lock_);
    if (cr_map_.find(cr->Id()) != cr_map_.end()) {
      return false;
    }
    cr_map_[cr->Id()] = cr;
  }

  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  cr->SetVRunningTime(min_vruntime_ + cr->NormalizedRunningTime());
  cr->SetExecTime(cr->VRunningTime() * cr->Priority());
  local_rb_map_.insert(
      std::pair<double, std::shared_ptr<CRoutine>>(cr->VRunningTime(), cr));
  return true;
}

bool CFSContext::EnqueueAffinityRoutine(const std::shared_ptr<CRoutine>& cr) {
  std::lock_guard<std::mutex> lg(rw_affinity_lock_);
  if (cr->IsAffinity(Id())) {
    affinity_rb_map_.insert(
        std::pair<uint32_t, std::shared_ptr<CRoutine>>(cr->Priority(), cr));
    return true;
  }
  return false;
}

bool CFSContext::RqEmpty() {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  return !notified_.load();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
