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
#include "cybertron/scheduler/policy/processor_context.h"

namespace apollo {
namespace cybertron {
namespace scheduler {

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
  Processor()
      : buffer_(true),
        main_thread_(this),
        emergency_thread_(this),
        routine_context_(new RoutineContext()) {}
  explicit Processor(bool buffer)
      : buffer_(buffer),
        main_thread_(this),
        emergency_thread_(this),
        routine_context_(new RoutineContext()) {}
  virtual ~Processor() {}

  void Start();
  void Stop();
  void BindContext(const std::shared_ptr<ProcessorContext>& context);
  void UpdateStat(ProcessorStat* processor_stat) {
    context_->UpdateProcessStat(&stat_);
    *processor_stat = stat_;
  }
  inline void Notify() { main_thread_.Notify(); }
  inline void NotifyEmergencyThread() {
    if (buffer_) {
      emergency_thread_.Notify();
    }
  }

  inline uint32_t Id() const { return id_; }
  inline void SetId(const uint32_t& id) { id_ = id; }
  inline const ProcessorStat& Stat() const { return stat_; }
  inline const std::shared_ptr<ProcessorContext>& Context() const {
    return context_;
  }
  inline bool EnableEmergencyThread() const { return buffer_; }

 private:
  class ProcessorThread {
   public:
    explicit ProcessorThread(Processor* processor) : processor_(processor) {}
    ~ProcessorThread() {
      if (thread_.joinable()) {
        thread_.join();
      }
    }
    void Start();
    void Run();
    void Notify() { cv_.notify_one(); }
    void SetHigherPriority();

   private:
    Processor* processor_;
    std::thread thread_;
    std::shared_ptr<CRoutine> cur_routine_ = nullptr;
    std::mutex cv_mutex_;
    std::condition_variable cv_;
    bool running_ = false;
    bool buffer_ = false;
  };

  ProcessorThread main_thread_;
  ProcessorThread emergency_thread_;
  std::shared_ptr<RoutineContext> routine_context_ = nullptr;
  std::shared_ptr<ProcessorContext> context_ = nullptr;
  bool running_ = false;
  bool buffer_ = false;
  uint32_t id_ = 0;
  ProcessorStat stat_;
};

}  // namespace scheduler
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_PROCESSOR_H_
