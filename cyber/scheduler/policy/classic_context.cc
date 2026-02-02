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

#include <cstdint>
#include <limits>

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::base::AtomicRWLock;
using apollo::cyber::base::ReadLockGuard;
using apollo::cyber::base::WriteLockGuard;
using apollo::cyber::croutine::CRoutine;
using apollo::cyber::croutine::RoutineState;

std::mutex ClassicContext::mtx_wq_;
GRP_WQ_CV ClassicContext::cv_wq_;
RQ_LOCK_GROUP ClassicContext::rq_locks_;
CR_GROUP ClassicContext::cr_group_;
NOTIFY_GRP ClassicContext::notify_grp_;

ClassicContext::ClassicContext() { InitGroup(DEFAULT_GROUP_NAME); }

ClassicContext::ClassicContext(const std::string& group_name) {
  InitGroup(group_name);
}

void ClassicContext::InitGroup(const std::string& group_name) {
  multi_pri_rq_ = &cr_group_[group_name];
  lq_ = &rq_locks_[group_name];
  cw_ = &cv_wq_[group_name];
  {
    std::lock_guard<std::mutex> lk(mtx_wq_);
    notify_grp_[group_name] = 0;
    current_grp = group_name;
  }
}

std::shared_ptr<CRoutine> ClassicContext::NextRoutine() {
  if (cyber_unlikely(stop_.load())) {
    return nullptr;
  }

  for (int i = MAX_PRIO - 1; i >= 0; --i) {
    ReadLockGuard<AtomicRWLock> lk(lq_->at(i));
    for (auto& cr : multi_pri_rq_->at(i)) {
      if (!cr->Acquire()) {
        continue;
      }

      if (cr->UpdateState() == RoutineState::READY) {
        return cr;
      }

      cr->Release();
    }
  }

  return nullptr;
}

void ClassicContext::Wait() {
  std::unique_lock<std::mutex> lk(mtx_wq_);
  cw_->Cv().wait_for(lk, std::chrono::milliseconds(1000),
                     [&]() { return notify_grp_[current_grp] > 0; });
  if (notify_grp_[current_grp] > 0) {
    notify_grp_[current_grp]--;
  }
}

void ClassicContext::Shutdown() {
  stop_.store(true);
  {
    std::lock_guard<std::mutex> lk(mtx_wq_);
    notify_grp_[current_grp] = std::numeric_limits<uint8_t>::max();
  }
  cw_->Cv().notify_all();
}

void ClassicContext::Notify(const std::string& group_name) {
  {
    std::lock_guard<std::mutex> lk(mtx_wq_);
    ++notify_grp_[group_name];
  }
  cv_wq_[group_name].Cv().notify_one();
}

bool ClassicContext::RemoveCRoutine(const std::shared_ptr<CRoutine>& cr) {
  auto grp = cr->group_name();
  auto prio = cr->priority();
  auto crid = cr->id();
  WriteLockGuard<AtomicRWLock> lk(ClassicContext::rq_locks_[grp].at(prio));
  auto& croutines = ClassicContext::cr_group_[grp].at(prio);
  for (auto it = croutines.begin(); it != croutines.end(); ++it) {
    if ((*it)->id() == crid) {
      auto cr = *it;
      cr->Stop();
      while (!cr->Acquire()) {
        std::this_thread::sleep_for(std::chrono::microseconds(1));
        AINFO_EVERY(1000) << "waiting for task " << cr->name() << " completion";
      }
      croutines.erase(it);
      cr->Release();
      return true;
    }
  }
  return false;
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
