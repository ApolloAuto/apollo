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

#ifndef CYBERTRON_CROUTINE_CROUTINE_H_
#define CYBERTRON_CROUTINE_CROUTINE_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cybertron/croutine/routine_context.h"

namespace apollo {
namespace cybertron {
namespace croutine {

using routine_t = uint32_t;
using CRoutineFunc = std::function<void()>;

enum class RoutineState {
  READY,
  RUNNING,
  FINISHED,
  IO_WAIT,
  SLEEP,
  WAITING_INPUT
};

struct RoutineStatistics {
  uint64_t switch_num = 0;
  double io_wait_time = 0;
  double sleep_time = 0;
  double exec_time = 0;
};

class CRoutine {
 public:
  explicit CRoutine(const CRoutineFunc &func);
  explicit CRoutine(CRoutineFunc &&func);
  virtual ~CRoutine();
  virtual void Routine();

  // static interface
  inline static void Yield() {
    SwapContext(GetCurrentRoutine()->GetContext(), GetMainContext());
  }

  // TODO(hewei03): weak pointer here?
  inline static void SetMainContext(
      const std::shared_ptr<RoutineContext> &context) {
    main_context_ = context;
  }

  inline static CRoutine *GetCurrentRoutine() { return current_routine_; }

  inline static RoutineContext *GetMainContext() { return main_context_.get(); }

  // public interface
  RoutineState Resume();

  inline RoutineContext *GetContext() { return &context_; }

  inline void Run() {
    func_();
    state_ = RoutineState::FINISHED;
  }

  inline void SetState(const RoutineState &state) { state_ = state; }

  inline const RoutineState &State() { return state_; }

  inline void Wake() { state_ = RoutineState::READY; }

  inline void HangUp() {
    state_ = RoutineState::IO_WAIT;
    CRoutine::Yield();
  }

  // TODO(hewei03): use cybertron::Time instead
  inline void Sleep(const useconds_t &usec) {
    state_ = RoutineState::SLEEP;
    wake_time_ =
        std::chrono::steady_clock::now() + std::chrono::microseconds(usec);
    statistic_info_.sleep_time += usec / 1000;
    CRoutine::Yield();
  }

  inline void SetId(uint64_t id) { id_ = id; }

  inline uint64_t Id() { return id_; }

  inline void SetProcessorId(int processor_id) { processor_id_ = processor_id; }

  inline int ProcessorId() { return processor_id_; }

  void PrintStatistics() const;
  RoutineStatistics GetStatistics() const;

  RoutineState UpdateState() {
    if (IsSleep()) {
      if (std::chrono::steady_clock::now() > wake_time_) {
        state_ = RoutineState::READY;
      }
    }
    return state_;
  }

  double VFrequency() { return vfrequency_; }
  void SetVFrequency(double frequency) { vfrequency_ = frequency; }

  double Frequency() { return frequency_; }
  void SetFrequency(double frequency) { frequency_ = frequency; }

  double NormalizedFrequency() { return normalized_vfrequency_; }
  void SetNormalizedFrequency(double frequency) {
    normalized_vfrequency_ = frequency;
  }

  double VRunningTime() { return vruntime_; }
  void SetVRunningTime(double time) { vruntime_ = time; }

  double NormalizedRunningTime() { return normalized_vruntime_; }
  void SetNormalizedRunningTime(double time) { normalized_vruntime_ = time; }

  uint32_t Priority() { return priority_; }
  void SetPriority(uint32_t priority) { priority_ = priority; }

  double ProcessedNum() { return proc_num_; }
  void SetProcessedNum(double num) { proc_num_ = num; }
  void IncreaseProcessedNum() { ++proc_num_; }

  bool IsRunning() { return state_ == RoutineState::RUNNING; }
  bool IsFinished() { return state_ == RoutineState::FINISHED; }
  bool IsWaitingInput() { return state_ == RoutineState::WAITING_INPUT; }
  bool IsReady() { return state_ == RoutineState::READY; }
  bool IsSleep() { return state_ == RoutineState::SLEEP; }
  bool IsIOWait() { return state_ == RoutineState::IO_WAIT; }

  void Stop() { force_stop_ = true; }

  std::chrono::steady_clock::time_point wake_time_;
  // for processor scheudle

  double notify_num_ = 0;

 private:
  uint64_t id_ = 0;
  uint32_t processor_id_ = 0;
  static thread_local CRoutine *current_routine_;
  static thread_local std::shared_ptr<RoutineContext> main_context_;
  RoutineContext context_;
  CRoutineFunc func_;
  RoutineStatistics statistic_info_;
  RoutineState state_;
  uint32_t priority_ = 1;
  uint64_t frequency_ = 0;
  double proc_num_ = 0;
  double normalized_vfrequency_ = 0.0;
  double vfrequency_ = 0.0;
  double vruntime_ = 0.0;
  double normalized_vruntime_ = 0.0;
  bool force_stop_ = false;
};

}  // namespace croutine
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_CROUTINE_CROUTINE_H_
