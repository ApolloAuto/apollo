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

#ifndef CYBERTRON_CROUTINE_CROUTINE_H_
#define CYBERTRON_CROUTINE_CROUTINE_H_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "cybertron/croutine/routine_context.h"

namespace apollo {
namespace cybertron {
namespace croutine {

using CRoutineFunc = std::function<void()>;
using Duration = std::chrono::microseconds;

enum class RoutineState {
  READY,
  RUNNING,
  FINISHED,
  IO_WAIT,
  SLEEP,
  WAITING_INPUT
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
    SetState(RoutineState::FINISHED);
  }

  inline bool TryLockForOp() {
    std::lock_guard<std::mutex> lh(op_mtx_);
    if (!being_op_) {
      being_op_ = true;
      return being_op_;
    }
    return false;
  }

  inline void TryUnlockForOp() {
    std::lock_guard<std::mutex> lh(op_mtx_);
    being_op_ = false;
  }

  inline void SetState(const RoutineState &state, bool is_notify = false) {
    // for race condition issue
    if (is_notify && state_.load() == RoutineState::RUNNING) {
      return;
    }
    state_.store(state);
  }

  inline const RoutineState State() { return state_.load(); }

  inline void Wake() { SetState(RoutineState::READY); }

  inline void HangUp() {
    SetState(RoutineState::IO_WAIT);
    CRoutine::Yield();
  }

  inline void Sleep(const Duration &sleep_duration) {
    SetState(RoutineState::SLEEP);
    wake_time_ = std::chrono::steady_clock::now() + sleep_duration;
    CRoutine::Yield();
  }

  inline void SetId(uint64_t id) { id_ = id; }
  inline uint64_t Id() { return id_; }

  inline void SetName(std::string name) { name_ = name; }
  inline std::string Name() { return name_; }

  inline void SetProcessorId(int processor_id) { processor_id_ = processor_id; }

  inline int ProcessorId() { return processor_id_; }

  void AddAffinityProcessor(int processor_id) {
    affinity_processor_id_set_.insert(processor_id);
  }

  std::set<int> &GetAffinityProcessorSet() {
    return affinity_processor_id_set_;
  }

  bool IsAffinity(int processor_id) {
    return affinity_processor_id_set_.find(processor_id) !=
           affinity_processor_id_set_.end();
  }

  RoutineState UpdateState() {
    if (IsSleep()) {
      if (std::chrono::steady_clock::now() > wake_time_) {
        SetState(RoutineState::READY);
      }
    }
    return state_.load();
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

  uint64_t ExecTime() { return exec_time_; }
  void SetExecTime(uint64_t time) { exec_time_ = time; }

  double NormalizedRunningTime() { return normalized_vruntime_; }
  void SetNormalizedRunningTime(double time) { normalized_vruntime_ = time; }

  uint32_t Priority() { return priority_; }
  void SetPriority(uint32_t priority) { priority_ = priority; }

  double ProcessedNum() { return proc_num_; }
  void SetProcessedNum(double num) { proc_num_ = num; }
  void IncreaseProcessedNum() { ++proc_num_; }

  bool IsRunning() { return state_.load() == RoutineState::RUNNING; }
  bool IsFinished() { return state_.load() == RoutineState::FINISHED; }
  bool IsWaitingInput() { return state_.load() == RoutineState::WAITING_INPUT; }
  bool IsReady() { return state_.load() == RoutineState::READY; }
  bool IsSleep() { return state_.load() == RoutineState::SLEEP; }
  bool IsIOWait() { return state_.load() == RoutineState::IO_WAIT; }

  void Stop() { force_stop_ = true; }

  std::chrono::steady_clock::time_point wake_time_;
  // for processor schedule

  double notify_num_ = 0;

 private:
  uint64_t id_ = 0;
  std::string name_;
  uint32_t processor_id_ = -1;
  std::set<int> affinity_processor_id_set_;

  static thread_local CRoutine *current_routine_;
  static thread_local std::shared_ptr<RoutineContext> main_context_;
  RoutineContext context_;
  CRoutineFunc func_;
  std::atomic<RoutineState> state_;
  uint32_t priority_ = 1;
  uint64_t frequency_ = 0;
  uint64_t exec_time_ = 0;
  double proc_num_ = 0;
  double normalized_vfrequency_ = 0.0;
  double vfrequency_ = 0.0;
  double vruntime_ = 0.0;
  double normalized_vruntime_ = 0.0;
  bool force_stop_ = false;
  bool being_op_ = false;
  std::mutex op_mtx_;
};

}  // namespace croutine
}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_CROUTINE_CROUTINE_H_
