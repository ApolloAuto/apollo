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

#include "cybertron/common/global_data.h"
#include "cybertron/common/log.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/croutine/routine_context.h"
#include "cybertron/scheduler/policy/processor_context.h"
#include "cybertron/scheduler/processor.h"
#include "cybertron/time/time.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

using apollo::cybertron::common::GlobalData;

void Processor::Start() {
  main_thread_.Start();
  if (buffer_) {
    emergency_thread_.Start();
    emergency_thread_.SetHigherPriority();
  }
}

void Processor::ProcessorThread::Start() {
  thread_ = std::thread(&ProcessorThread::Run, this);
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(processor_->id_, &set);
  if (pthread_setaffinity_np(thread_.native_handle(), sizeof(set), &set) != 0) {
    AWARN << "Set cpu affinity failed!" << std::endl;
  }
}

void Processor::ProcessorThread::Run() {
  CRoutine::SetMainContext(processor_->routine_context_);
  processor_->running_ = true;
  while (processor_->running_) {
    // there is a lock in NextRoutine
    // std::unique_lock<std::mutex> context_lock(processor_->context_mutex_);
    if (processor_->context_) {
      cur_routine_ = processor_->context_->NextRoutine();
      if (cur_routine_) {
        if (cur_routine_->Resume() ==
            croutine::RoutineState::FINISHED) {
          processor_->context_->RemoveCRoutine(cur_routine_->Id());
        }

        cur_routine_ = nullptr;
        if (processor_->buffer_) {
          std::unique_lock<std::mutex> lk_rq(cv_mutex_);
          cv_.wait(lk_rq);
        }
      } else {
        std::unique_lock<std::mutex> lk_rq(cv_mutex_);
        if (processor_->buffer_) {
          cv_.wait(lk_rq);
        } else {
          cv_.wait_for(lk_rq, std::chrono::milliseconds(1));
          // cv_.wait(lk_rq, [this] {
          //   return !this->processor_->context_->RqEmpty();
          // });
        }
      }
    } else {
      // context_lock.unlock();
      AINFO << "no pcontext bound, wait..." << std::endl;
      std::unique_lock<std::mutex> lk_rq(cv_mutex_);
      cv_.wait(lk_rq, [this] {
        return this->processor_->context_ &&
               !this->processor_->context_->RqEmpty();
      });
    }
  }
}

void Processor::ProcessorThread::SetHigherPriority() {
  sched_param sch;
  int policy = 0;
  pthread_getschedparam(thread_.native_handle(), &policy, &sch);
  sch.sched_priority = sch.sched_priority + 20;
  if (pthread_setschedparam(thread_.native_handle(), SCHED_RR, &sch)) {
    AWARN << "Failed to setschedparam: " << std::strerror(errno);
  }
}

void Processor::UpdateStat(ProcessorStat* processor_stat) {
  context_->UpdateProcessStat(&stat_);
  *processor_stat = stat_;
}

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo
