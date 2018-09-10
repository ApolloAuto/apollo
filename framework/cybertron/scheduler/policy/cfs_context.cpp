/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "cybertron/scheduler/policy/cfs_context.h"

#include "cybertron/common/log.h"
#include "cybertron/common/types.h"
#include "cybertron/common/global_data.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/scheduler/processor.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using croutine::RoutineState;
using apollo::cybertron::common::GlobalData;

std::shared_ptr<CRoutine> CFSContext::NextRoutine() {
  if (stop_) {
    return nullptr;
  }

  auto start_time = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mtx_run_queue_);

  for (auto it = run_queue_.begin();
       it != run_queue_.end();) {
    auto cr = (*it);
    cr->SetVRunningTime(cr->GetStatistics().exec_time / cr->Priority());
    rb_map_.insert(std::pair<double, std::shared_ptr<CRoutine>>(cr->VRunningTime(), cr));
    it = run_queue_.erase(it);
  }

  for (auto it = wait_queue_.begin();
       it != wait_queue_.end();) {
    auto cr = (*it);
    cr->UpdateState();
    if (cr->IsReady()) {
      cr->SetVRunningTime(min_vruntime_ -
                                cr->NormalizedRunningTime());
      rb_map_.insert(std::pair<double, std::shared_ptr<CRoutine>>(cr->VRunningTime(), cr));
      it = wait_queue_.erase(it);
      continue;
    }
    ++it;
  }

  std::shared_ptr<CRoutine> croutine = nullptr;
  for (auto it = rb_map_.begin(); it != rb_map_.end();) {
    auto cr = it->second;
    //AINFO << GlobalData::GetTaskNameById(cr->Id()) << " " << cr->GetStatistics().exec_time / cr->Priority();
    auto cr_id = cr->Id();
    std::promise<std::shared_ptr<CRoutine>>* promise = nullptr;
    if (pop_list_.Get(cr_id, &promise)) {
      cr->SetNormalizedRunningTime(cr->VRunningTime() - min_vruntime_);
      promise->set_value(cr);
      pop_list_.Remove(cr_id);
      it = rb_map_.erase(it);
      continue;
    }

    if (cr->IsFinished()) {
      it = rb_map_.erase(it);
      continue;
    }

    // TODO: We should use asynchronous notification mechanism later.
    if (cr->IsSleep() || cr->IsWaitingInput()) {
      wait_queue_.emplace_back(cr);
      AINFO << "pop from rq routine[" << cr_id << "] size["
             << wait_queue_.size() << "]";
      it = rb_map_.erase(it);
      continue;
    }

    if (cr->IsReady()) {
        min_vruntime_ = cr->VRunningTime();
        croutine = cr;
        run_queue_.emplace_back(cr);
        rb_map_.erase(it);
        break;
    }
    ++it;
  }
  proc_interval_ += std::chrono::steady_clock::now() - start_time;
  proc_num_++;
  if (croutine == nullptr) {
    notified_.store(false);
  }
  return croutine;
}

bool CFSContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  cr->SetVRunningTime(min_vruntime_ + cr->NormalizedRunningTime());
  rb_map_.insert(std::pair<double, std::shared_ptr<CRoutine>>(cr->VRunningTime(), cr));
  return true;
}

bool CFSContext::IsPriorInverse(uint64_t routine_id) { return false; }

bool CFSContext::RqEmpty() {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  return !notified_.load();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
