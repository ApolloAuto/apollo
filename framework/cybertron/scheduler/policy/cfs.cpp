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

using apollo::cybertron::common::GlobalData;
using apollo::cybertron::event::PerfEventCache;
using apollo::cybertron::event::SchedPerf;
using apollo::cybertron::proto::RoutineConf;
using apollo::cybertron::proto::RoutineConfInfo;
using apollo::cybertron::proto::SchedulerConf;
using croutine::RoutineState;

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
    cur_croutine_->set_vruntime(
        static_cast<double>(cur_croutine_->exec_time()) /
        cur_croutine_->priority());
    cur_croutine_->set_exec_time(cur_croutine_->vruntime() *
                                 cur_croutine_->priority());
    local_rb_map_.insert(std::pair<double, std::shared_ptr<CRoutine>>(
        cur_croutine_->vruntime(), cur_croutine_));
    cur_croutine_ = nullptr;
  }

  std::shared_ptr<CRoutine> croutine = nullptr;
  for (auto it = local_rb_map_.begin(); it != local_rb_map_.end();) {
    auto cr = it->second;
    auto lock = cr->TryLock();
    if (!lock) {
      ++it;
      continue;
    }

    cr->UpdateState();
    if (cr->state() == RoutineState::RUNNING) {
      ++it;
      continue;
    }

    if (cr->state() == RoutineState::FINISHED) {
      it = local_rb_map_.erase(it);
      continue;
    }

    if (cr->state() == RoutineState::READY) {
      min_vruntime_ = cr->vruntime();
      croutine = cr;
      croutine->set_state(RoutineState::RUNNING);
      cur_croutine_ = croutine;
      local_rb_map_.erase(it);
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

std::shared_ptr<CRoutine> CFSContext::NextAffinityRoutine() {
  if (stop_) {
    return nullptr;
  }
  std::lock_guard<std::mutex> lg(rw_affinity_lock_);
  auto start_perf_time = apollo::cybertron::Time::Now().ToNanosecond();
  std::shared_ptr<CRoutine> croutine = nullptr;

  for (auto it = affinity_rb_map_.begin(); it != affinity_rb_map_.end();) {
    auto cr = it->second;
    auto lock = cr->TryLock();
    if (!lock) {
      ++it;
      continue;
    }

    cr->UpdateState();
    if (cr->state() == RoutineState::FINISHED) {
      it = affinity_rb_map_.erase(it);
      continue;
    }

    if (cr->state() == RoutineState::RUNNING) {
      ++it;
      continue;
    }

    if (cr->state() == RoutineState::READY) {
      croutine = cr;
      cr->set_state(RoutineState::RUNNING);
      break;
    }
    ++it;
  }
  if (croutine) {
    PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NEXT_AFFINITY_R,
                                              croutine->id(), proc_index_, 0,
                                              start_perf_time, -1, -1);
  }
  return croutine;
}

bool CFSContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  if (cr->processor_id() != id()) {
    return false;
  }

  {
    WriteLockGuard<AtomicRWLock> lg(rw_lock_);
    if (cr_map_.find(cr->id()) != cr_map_.end()) {
      return false;
    }
    cr_map_[cr->id()] = cr;
  }

  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  cr->set_vruntime(min_vruntime_ + cr->normalized_vruntime());
  cr->set_exec_time(cr->vruntime() * cr->priority());
  local_rb_map_.insert(
      std::pair<double, std::shared_ptr<CRoutine>>(cr->vruntime(), cr));
  return true;
}

bool CFSContext::EnqueueAffinityRoutine(const std::shared_ptr<CRoutine>& cr) {
  std::lock_guard<std::mutex> lg(rw_affinity_lock_);
  if (cr->IsAffinity(id())) {
    affinity_rb_map_.insert(
        std::pair<uint32_t, std::shared_ptr<CRoutine>>(cr->priority(), cr));
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
