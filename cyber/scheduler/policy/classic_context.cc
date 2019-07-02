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

#include "cyber/scheduler/policy/classic_context.h"

#include "cyber/event/perf_event_cache.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::croutine::CRoutine;
using apollo::cyber::croutine::RoutineState;
using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

alignas(CACHELINE_SIZE) GRP_WQ_MUTEX ClassicContext::mtx_wq_;
alignas(CACHELINE_SIZE) GRP_WQ_CV ClassicContext::cv_wq_;
alignas(CACHELINE_SIZE) RQ_LOCK_GROUP ClassicContext::rq_locks_;
alignas(CACHELINE_SIZE) CR_GROUP ClassicContext::cr_group_;

ClassicContext::ClassicContext() { InitGroup(DEFAULT_GROUP_NAME); }

ClassicContext::ClassicContext(const std::string& group_name) {
  InitGroup(group_name);
}

void ClassicContext::InitGroup(const std::string& group_name) {
  multi_pri_rq_ = &cr_group_[group_name];
  lq_ = &rq_locks_[group_name];
  mtx_wrapper_ = &mtx_wq_[group_name];
  cw_ = &cv_wq_[group_name];
}

std::shared_ptr<CRoutine> ClassicContext::NextRoutine() {
  if (unlikely(stop_)) {
    return nullptr;
  }

  for (int i = MAX_PRIO - 1; i >= 0; --i) {
    ReadLockGuard<AtomicRWLock> lk(lq_->at(i));
    for (auto& cr : multi_pri_rq_->at(i)) {
      if (!cr->Acquire()) {
        continue;
      }

      if (cr->UpdateState() == RoutineState::READY) {
        PerfEventCache::Instance()->AddSchedEvent(SchedPerf::NEXT_RT, cr->id(),
                                                  cr->processor_id());
        return cr;
      }

      if (unlikely(cr->state() == RoutineState::SLEEP)) {
        if (!need_sleep_ || wake_time_ > cr->wake_time()) {
          need_sleep_ = true;
          wake_time_ = cr->wake_time();
        }
      }

      cr->Release();
    }
  }

  return nullptr;
}

void ClassicContext::Wait() {
  std::unique_lock<std::mutex> lk(mtx_wrapper_->Mutex());
  if (stop_) {
    return;
  }

  if (unlikely(need_sleep_)) {
    auto duration = wake_time_ - std::chrono::steady_clock::now();
    cw_->Cv().wait_for(lk, duration);
    need_sleep_ = false;
  } else {
    cw_->Cv().wait(lk);
  }
}

void ClassicContext::Shutdown() {
  {
    std::lock_guard<std::mutex> lg(mtx_wrapper_->Mutex());
    if (!stop_) {
      stop_ = true;
    }
  }
  cw_->Cv().notify_all();
}

void ClassicContext::Notify(const std::string& group_name) {
  cv_wq_[group_name].Cv().notify_one();
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
