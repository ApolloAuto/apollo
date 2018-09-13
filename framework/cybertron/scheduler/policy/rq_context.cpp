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

#include "cybertron/scheduler/policy/rq_context.h"

#include "cybertron/common/log.h"
#include "cybertron/common/types.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/scheduler/processor.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using croutine::RoutineState;

std::shared_ptr<CRoutine> RQContext::NextRoutine() {
  {
    std::lock_guard<std::mutex> lock(mtx_run_queue_);
    if (run_queue_.empty() || stop_) {
      return nullptr;
    }

    if (run_queue_it_ == run_queue_.end()) {
      run_queue_it_ = run_queue_.begin();
    }

    for (; run_queue_it_ != run_queue_.end();) {
      auto rt = *run_queue_it_;
      auto rt_id = rt->Id();
      std::promise<std::shared_ptr<CRoutine>>* promise = nullptr;
      if (pop_list_.Get(rt_id, &promise)) {
        promise->set_value(rt);
        pop_list_.Remove(rt_id);
        run_queue_it_ = run_queue_.erase(run_queue_it_);
        continue;
      }

      if (rt->IsFinished()) {
        run_queue_it_ = run_queue_.erase(run_queue_it_);
        continue;
      }

      // TODO: We should use asynchronous notification mechanism later.
      rt->UpdateState();
      if (rt->IsReady()) {
        auto routine = *run_queue_it_;
        routine->SetState(RoutineState::RUNNING);
        ++run_queue_it_;
        return routine;
      }
      ++run_queue_it_;
    }
  }
  notified_.store(false);
  return nullptr;
}

bool RQContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  auto itr = run_queue_.begin();
  for (auto itr = run_queue_.begin(); itr != run_queue_.end(); ++itr) {
    if ((*itr)->Priority() < cr->Priority()) {
      run_queue_.insert(itr, cr);
      return true;
    }
  }
  run_queue_.push_back(cr);
  return true;
}

bool RQContext::IsPriorInverse(uint64_t routine_id) {
  int wait_index = -1;
  auto index = 0;
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  for (auto& cr : run_queue_) {
    if (cr->Id() == routine_id) {
      if (cr->IsRunning()) {
        return false;
      }
      wait_index = index;
      continue;
    }

    if (cr->IsRunning()) {
      if (wait_index == -1) {
        return false;
      } else if (wait_index < index) {
        return true;
      }
    }
    ++index;
  }
  return false;
}

bool RQContext::RqEmpty() {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  return !notified_.load() || run_queue_.empty();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
