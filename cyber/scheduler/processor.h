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

#ifndef CYBER_SCHEDULER_PROCESSOR_H_
#define CYBER_SCHEDULER_PROCESSOR_H_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "cyber/croutine/croutine.h"
#include "cyber/croutine/routine_context.h"
#include "cyber/proto/scheduler_conf.pb.h"

namespace apollo {
namespace cyber {
namespace scheduler {

class ProcessorContext;

using croutine::CRoutine;
using croutine::RoutineContext;
using apollo::cyber::proto::SchedStrategy;

class Processor {
 public:
  Processor();
  ~Processor();

  void Run();
  void Start();
  void Stop() {
    running_.exchange(false);
    Notify();
  }
  void Notify() { cv_.notify_one(); }

  void BindContext(const std::shared_ptr<ProcessorContext>& context) {
    context_ = context;
  }

  uint32_t Id() const { return id_; }
  void SetId(uint32_t id) { id_ = id; }
  void SetCpuBindingStartIndex(uint32_t start_index) {
    cpu_binding_start_index_ = start_index;
  }

  void SetStrategy(const SchedStrategy strategy) {
    strategy_ = strategy;
  }

 private:
  std::mutex mtx_rq_;
  std::mutex mtx_pctx_;
  std::condition_variable cv_;

  std::thread thread_;

  SchedStrategy strategy_ = SchedStrategy::CHOREO;

  std::shared_ptr<ProcessorContext> context_;
  std::shared_ptr<RoutineContext> routine_context_ = nullptr;

  std::atomic<bool> running_{false};
  uint32_t id_ = 0;
  uint32_t cpu_binding_start_index_ = 0;
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_PROCESSOR_H_
