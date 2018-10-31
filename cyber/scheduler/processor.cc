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

#include <sched.h>
#include <chrono>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/croutine/croutine.h"
#include "cyber/croutine/routine_context.h"
#include "cyber/event/perf_event_cache.h"
#include "cyber/scheduler/processor.h"
#include "cyber/scheduler/processor_context.h"
#include "cyber/scheduler/scheduler.h"

namespace apollo {
namespace cyber {
namespace scheduler {

using apollo::cyber::event::PerfEventCache;
using apollo::cyber::event::SchedPerf;

Processor::Processor() { routine_context_.reset(new RoutineContext()); }

Processor::~Processor() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

void Processor::Start() {
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&Processor::Run, this);

  uint32_t core_num = std::thread::hardware_concurrency();
  if (strategy_ != SchedStrategy::CLASSIC && core_num != 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(id_, &set);
    pthread_setaffinity_np(thread_.native_handle(), sizeof(set), &set);
  }
}

void Processor::Run() {
  CRoutine::SetMainContext(routine_context_);
  while (likely(running_)) {
    if (likely(context_ != nullptr)) {
      auto croutine = context_->NextRoutine();
      if (croutine) {
        croutine->Resume();
        croutine->Release();
      } else {
        std::unique_lock<std::mutex> ul(mtx_rq_);
        cv_.wait_for(ul, std::chrono::milliseconds(1));
      }
    } else {
      std::unique_lock<std::mutex> lk_rq(mtx_rq_);
      cv_.wait(lk_rq, [this] { return !this->running_; });
    }
  }
}

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo
