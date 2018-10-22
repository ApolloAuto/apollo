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
using apollo::cyber::proto::ProcessStrategy;

class Processor {
 public:
  Processor();
  virtual ~Processor();

  void Run();
  void Start();
  void Stop() {
    running_ = false;
    Notify();
  }
  void Notify() { cv_.notify_one(); }

  inline void bind_context(const std::shared_ptr<ProcessorContext>& context) {
    context_ = context;
  }

  inline uint32_t id() const { return id_; }
  inline void set_id(uint32_t id) { id_ = id; }


  void set_strategy(const ProcessStrategy strategy) { strategy_ = strategy; }

 private:
  std::mutex mtx_rq_;
  std::mutex mtx_pctx_;
  std::condition_variable cv_;

  std::thread thread_;

  std::shared_ptr<RoutineContext> routine_context_ = nullptr;
  std::shared_ptr<CRoutine> cur_routine_ = nullptr;
  std::shared_ptr<ProcessorContext> context_;

  bool running_ = true;
  uint32_t id_ = 0;

  ProcessStrategy strategy_ = ProcessStrategy::CHOREO;
};

}  // namespace scheduler
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_SCHEDULER_PROCESSOR_H_
