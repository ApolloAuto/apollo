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

#ifndef CYBERTRON_SCHEDULER_PROCESSOR_H_
#define CYBERTRON_SCHEDULER_PROCESSOR_H_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "cybertron/croutine/croutine.h"
#include "cybertron/croutine/routine_context.h"
#include "cybertron/proto/scheduler_conf.pb.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

class ProcessorContext;

using croutine::CRoutine;
using croutine::RoutineContext;

struct ProcessorStat {
  uint64_t switch_num = 0;
  double io_wait_time = 0;
  double sleep_time = 0;
  double exec_time = 0;
};

class Processor {
 public:
  Processor();
  virtual ~Processor();

  void Run();
  void Start();
  void BindContext(const std::shared_ptr<ProcessorContext>& context) {
    context_ = context;
  }
  void set_id(uint32_t id) { id_ = id; }
  uint32_t id() const { return id_; }
  void Stop() { running_ = false; }
  void Notify() { cv_.notify_one(); }

  std::shared_ptr<ProcessorContext> Context() { return context_; }

 private:
  std::thread thread_;
  std::mutex mtx_rq_;
  std::mutex mtx_pctx_;
  std::condition_variable cv_;
  std::shared_ptr<RoutineContext> routine_context_ = nullptr;
  std::shared_ptr<CRoutine> cur_routine_ = nullptr;
  std::shared_ptr<ProcessorContext> context_ = nullptr;
  uint32_t id_ = 0;
  bool running_ = true;
};

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_PROCESSOR_H_
