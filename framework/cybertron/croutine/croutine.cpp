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

#include "cybertron/croutine/croutine.h"

#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/croutine/routine_context.h"
#include "cybertron/event/perf_event_cache.h"

namespace apollo {
namespace cybertron {
namespace croutine {

using apollo::cybertron::event::PerfEventCache;

thread_local CRoutine *CRoutine::current_routine_;
thread_local std::shared_ptr<RoutineContext> CRoutine::main_context_;

static void CRoutineEntry(void *arg) {
  CRoutine *r = static_cast<CRoutine *>(arg);
  r->Run();
  SwapContext(r->GetContext(), CRoutine::GetMainContext());
}

CRoutine::CRoutine(const std::function<void()> &func) {
  func_ = func;
  MakeContext(CRoutineEntry, this, &context_);
  state_ = RoutineState::READY;
}

CRoutine::CRoutine(std::function<void()> &&func) {
  func_ = std::move(func);
  MakeContext(CRoutineEntry, this, &context_);
  state_ = RoutineState::READY;
}

void CRoutine::PrintStatistics() const {
  AINFO << "routine_id[" << id_ << "] exec_time[" << statistic_info_.exec_time
        << "] sleep_time[" << statistic_info_.sleep_time << "] switch_num["
        << statistic_info_.switch_num << "]";
}

CRoutine::~CRoutine() { PrintStatistics(); }

RoutineState CRoutine::Resume() {
  if (force_stop_) {
    state_ = RoutineState::FINISHED;
    return state_;
  }

  UpdateState();

  // Keep compatibility with different policies.
  if (!IsRunning() && !IsReady()) {
    if (IsWaitingInput()) {
      AERROR << "Wait input";
    }
    AERROR << "Invalid Routine State!";
    return state_;
  }

  current_routine_ = this;
  // update statistics info
  auto t_start = std::chrono::high_resolution_clock::now();
  PerfEventCache::Instance()->AddSchedEvent(1, id_, processor_id_, 0, 0);
  SwapContext(GetMainContext(), this->GetContext());
  if (IsRunning()) {
    state_ = RoutineState::READY;
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  auto start_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         t_start.time_since_epoch())
                         .count();
  PerfEventCache::Instance()->AddSchedEvent(2, id_, processor_id_, 0,
                                            start_nanos);
  auto diff =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
  statistic_info_.exec_time += diff;
  statistic_info_.switch_num++;
  return state_;
}

void CRoutine::Routine() {
  while (true) {
    AINFO << "inner routine" << std::endl;
    usleep(1000000);
  }
}

RoutineStatistics CRoutine::GetStatistics() const {
  // TODO(hewei03): We need a mutex if try to clear statistics info later.
  return statistic_info_;
}

}  // namespace croutine
}  // namespace cybertron
}  // namespace apollo
