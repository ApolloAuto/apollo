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

#include "cybertron/scheduler/policy/fcfs_context.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <utility>

#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/common/types.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/event/perf_event_cache.h"
#include "cybertron/scheduler/processor.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::common::GlobalData;
using apollo::cybertron::event::PerfEventCache;
using apollo::cybertron::event::SchedPerf;
using croutine::RoutineState;

std::shared_ptr<CRoutine> FCFSContext::NextRoutine() {
  if (stop_) {
    return nullptr;
  }
  auto start_perf_time = apollo::cybertron::Time::Now().ToNanosecond();
  std::shared_ptr<CRoutine> routine = nullptr;
  double min_vfrequency = -1;
  {
    std::lock_guard<std::mutex> lg(mtx_run_queue_);

    auto run_queue_it = run_queue_.begin();
    while (run_queue_it != run_queue_.end()) {
      auto& rt = *run_queue_it;
      auto rt_id = rt->Id();

      if (rt->IsRunning() || rt->IsWaitingInput()) {
        ++run_queue_it;
        continue;
      }

      std::promise<std::shared_ptr<CRoutine>>* promise = nullptr;
      if (pop_list_.Get(rt_id, &promise)) {
        rt->SetNormalizedFrequency(rt->VFrequency() - min_vfrequency_);
        promise->set_value(rt);
        pop_list_.Remove(rt_id);
        run_queue_it = run_queue_.erase(run_queue_it);
        continue;
      }
      rt->UpdateState();
      if (rt->IsReady()) {
        if (min_vfrequency < 0 || rt->VFrequency() < min_vfrequency) {
          min_vfrequency = rt->VFrequency();
          routine = rt;
        }
        ++run_queue_it;
        continue;
      }
      ++run_queue_it;
    }
  }
  if (routine == nullptr) {
    notified_.store(false);
  } else {
    min_vfrequency_ = min_vfrequency;
    routine->IncreaseProcessedNum();
    routine->SetVFrequency(routine->ProcessedNum() / routine->Frequency());
    routine->SetState(RoutineState::RUNNING);
    PerfEventCache::Instance()->AddSchedEvent(
        SchedPerf::NEXT_ROUTINE, routine->Id(), routine->ProcessorId(), 0,
        start_perf_time, -1, -1);
  }
  return routine;
}

bool FCFSContext::IsPriorInverse(uint64_t routine_id) {
  int wait_index = -1;
  auto index = 0;
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  for (auto& cr : run_queue_) {
    if (cr->Id() == routine_id && cr->VFrequency() <= min_vfrequency_) {
      if (cr->IsRunning()) {
        return false;
      }
      wait_index = index;
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

bool FCFSContext::Enqueue(const std::shared_ptr<CRoutine>& cr) {
  cr->SetVFrequency(min_vfrequency_ + cr->NormalizedFrequency());
  cr->SetProcessedNum(1);
  if (cr->Frequency() <= 0) {
    cr->SetFrequency(1);
  }

  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  auto itr = run_queue_.begin();
  for (auto itr = run_queue_.begin(); itr != run_queue_.end(); ++itr) {
    if ((*itr)->Priority() < cr->Priority()) {
      run_queue_.insert(itr, cr);
      return true;
    }
  }
  // TODO rb-tree needed here
  run_queue_.emplace_back(cr);
  return true;
}

bool FCFSContext::RqEmpty() {
  std::lock_guard<std::mutex> lg(mtx_run_queue_);
  return !notified_.load() || run_queue_.empty();
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
