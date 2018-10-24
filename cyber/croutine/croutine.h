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

#ifndef CYBER_CROUTINE_CROUTINE_H_
#define CYBER_CROUTINE_CROUTINE_H_

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "cyber/croutine/routine_context.h"

namespace apollo {
namespace cyber {
namespace croutine {

using RoutineFunc = std::function<void()>;
using Duration = std::chrono::microseconds;

enum class RoutineState { READY, RUNNING, FINISHED, SLEEP, IO_WAIT, DATA_WAIT };

class CRoutine {
 public:
  explicit CRoutine(const RoutineFunc &func);
  explicit CRoutine(RoutineFunc &&func);
  virtual ~CRoutine();
  virtual void Routine();

  // static interfaces
  static void Yield();
  static void Yield(const RoutineState &state);
  static void SetMainContext(const std::shared_ptr<RoutineContext> &context);
  static CRoutine *GetCurrentRoutine();
  static RoutineContext *GetMainContext();

  // public interfaces
  RoutineState Resume();
  RoutineState UpdateState();
  RoutineContext *GetContext();
  std::unique_lock<std::mutex> GetLock() const;
  std::unique_lock<std::mutex> TryLock() const;
  std::unique_lock<std::mutex> DeferLock() const;

  void Run();
  void Stop();
  void Wake();
  void HangUp();
  void Sleep(const Duration &sleep_duration);

  // getter and setter
  RoutineState state() const;
  void set_state(const RoutineState &state);

  uint64_t id() const;
  void set_id(uint64_t id);

  const std::string &name() const;
  void set_name(std::string name);

  int processor_id() const;
  void set_processor_id(int processor_id);

  double vfrequency() const;
  void set_vfrequency(double frequency);

  double frequency() const;
  void set_frequency(double frequency);

  double normalized_vfrequency() const;
  void set_normalized_vfrequency(double frequency);

  double vruntime() const;
  void set_vruntime(double time);

  uint64_t exec_time() const;
  void set_exec_time(uint64_t time);

  double normalized_vruntime() const;
  void set_normalized_vruntime(double time);

  uint32_t priority() const;
  void set_priority(uint32_t priority);

  double proc_num() const;
  void set_proc_num(double num);

 private:
  void Lock();
  void Unlock();

  std::string name_;
  std::chrono::steady_clock::time_point wake_time_;

  RoutineFunc func_;
  RoutineState state_;
  RoutineContext context_;

  mutable std::mutex mutex_;
  std::unique_lock<std::mutex> lock_;

  bool force_stop_ = false;
  double proc_num_ = 0.0;
  double vruntime_ = 0.0;
  double frequency_ = 1.0;
  double vfrequency_ = 0.0;
  double normalized_vruntime_ = 0.0;
  double normalized_vfrequency_ = 0.0;
  int processor_id_ = -1;
  uint32_t priority_ = 1;
  uint64_t id_ = 0;
  uint64_t exec_time_ = 0;

  static thread_local CRoutine *current_routine_;
  static thread_local std::shared_ptr<RoutineContext> main_context_;
};

inline void CRoutine::Yield(const RoutineState &state) {
  auto routine = GetCurrentRoutine();
  routine->Lock();
  routine->set_state(state);
  SwapContext(GetCurrentRoutine()->GetContext(), GetMainContext());
}

inline void CRoutine::Yield() {
  SwapContext(GetCurrentRoutine()->GetContext(), GetMainContext());
}

inline CRoutine *CRoutine::GetCurrentRoutine() { return current_routine_; }

inline RoutineContext *CRoutine::GetMainContext() {
  return main_context_.get();
}

inline void CRoutine::SetMainContext(
    const std::shared_ptr<RoutineContext> &context) {
  main_context_ = context;
}

inline RoutineContext *CRoutine::GetContext() { return &context_; }
RoutineState Resume();

inline void CRoutine::Run() { func_(); }

inline std::unique_lock<std::mutex> CRoutine::GetLock() const {
  std::unique_lock<std::mutex> ul(mutex_);
  return ul;
}

inline std::unique_lock<std::mutex> CRoutine::DeferLock() const {
  std::unique_lock<std::mutex> ul(mutex_, std::defer_lock);
  return ul;
}

inline std::unique_lock<std::mutex> CRoutine::TryLock() const {
  std::unique_lock<std::mutex> ul(mutex_, std::try_to_lock);
  return ul;
}

inline void CRoutine::set_state(const RoutineState &state) { state_ = state; }

inline RoutineState CRoutine::state() const { return state_; }

inline void CRoutine::Wake() { state_ = RoutineState::READY; }

inline void CRoutine::HangUp() { CRoutine::Yield(RoutineState::DATA_WAIT); }

inline void CRoutine::Sleep(const Duration &sleep_duration) {
  wake_time_ = std::chrono::steady_clock::now() + sleep_duration;
  CRoutine::Yield(RoutineState::SLEEP);
}

inline uint64_t CRoutine::id() const { return id_; }

inline void CRoutine::set_id(uint64_t id) { id_ = id; }

inline const std::string &CRoutine::name() const { return name_; }

inline void CRoutine::set_name(std::string name) { name_ = name; }

inline int CRoutine::processor_id() const { return processor_id_; }

inline void CRoutine::set_processor_id(int processor_id) {
  processor_id_ = processor_id;
}

inline RoutineState CRoutine::UpdateState() {
  if (state_ == RoutineState::SLEEP &&
      std::chrono::steady_clock::now() > wake_time_) {
    state_ = RoutineState::READY;
  }
  return state_;
}

inline double CRoutine::vfrequency() const { return vfrequency_; }

inline void CRoutine::set_vfrequency(double frequency) {
  vfrequency_ = frequency;
}

inline double CRoutine::frequency() const { return frequency_; }

inline void CRoutine::set_frequency(double frequency) {
  frequency_ = frequency;
}

inline double CRoutine::normalized_vfrequency() const {
  return normalized_vfrequency_;
}

inline void CRoutine::set_normalized_vfrequency(double frequency) {
  normalized_vfrequency_ = frequency;
}

inline double CRoutine::vruntime() const { return vruntime_; }

inline void CRoutine::set_vruntime(double time) { vruntime_ = time; }

inline uint64_t CRoutine::exec_time() const { return exec_time_; }

inline void CRoutine::set_exec_time(uint64_t time) { exec_time_ = time; }

inline double CRoutine::normalized_vruntime() const {
  return normalized_vruntime_;
}

inline void CRoutine::set_normalized_vruntime(double time) {
  normalized_vruntime_ = time;
}

inline uint32_t CRoutine::priority() const { return priority_; }

inline void CRoutine::set_priority(uint32_t priority) { priority_ = priority; }

inline double CRoutine::proc_num() const { return proc_num_; }

inline void CRoutine::set_proc_num(double num) { proc_num_ = num; }

inline void CRoutine::Lock() { lock_ = std::unique_lock<std::mutex>(mutex_); }

inline void CRoutine::Unlock() {
  if (lock_) {
    lock_.unlock();
  }
}

}  // namespace croutine
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_CROUTINE_CROUTINE_H_
