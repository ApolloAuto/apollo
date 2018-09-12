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

#ifndef CYBERTRON_SCHEDULER_TASK_H_
#define CYBERTRON_SCHEDULER_TASK_H_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "cybertron/common/global_data.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/data/data_dispatcher.h"
#include "cybertron/init.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {

static const char* task_prefix = "/internal/task/";

template <typename T = void>
class Task {
 public:
  template <typename Function>
  Task(const std::string& name, Function&& f, const uint8_t& num_threads = 1);
  ~Task();

  void Execute(const std::shared_ptr<T>& val);
  void Finish();
  template <typename Rep, typename Period>
  bool WaitFor(const std::chrono::duration<Rep, Period>& time);
  bool Wait();

 private:
  uint32_t num_threads_;
  std::atomic<uint32_t> running_count_;
  std::string name_;
  std::vector<uint64_t> name_ids_;
  std::mutex count_lock_;
};

template <>
template <typename Function>
Task<void>::Task(const std::string& name, Function&& f,
                 const uint8_t& num_threads)
    : name_(name), num_threads_(num_threads), running_count_(0) {
  auto channel_name = task_prefix + name;
  croutine::RoutineFactory factory = croutine::CreateRoutineFactory(f);
  scheduler::Scheduler::Instance()->CreateTask(factory, channel_name);
}

template <typename T>
template <typename Function>
Task<T>::Task(const std::string& name, Function&& f, const uint8_t& num_threads)
    : name_(name), num_threads_(num_threads), running_count_(0) {
  auto func =
      [ f = std::forward<Function&&>(f), this](const std::shared_ptr<T>& msg) {
    f(msg);
    this->Finish();
  };
  name_ids_.reserve(num_threads_);
  for (int i = 0; i < num_threads; ++i) {
    auto channel_name = task_prefix + name_ + std::to_string(i);
    auto name_id = common::GlobalData::RegisterChannel(channel_name);
    name_ids_.push_back(std::move(name_id));
    auto dv = std::make_shared<data::DataVisitor<T>>(name_id, 1);
    croutine::RoutineFactory factory = croutine::CreateRoutineFactory<T>(func, dv);
    scheduler::Scheduler::Instance()->CreateTask(factory, channel_name);
  }
}

template <typename T>
Task<T>::~Task() {
  Wait();
  for (int i = 0; i < num_threads_; ++i) {
    auto channel_name = task_prefix + name_ + std::to_string(i);
    scheduler::Scheduler::Instance()->RemoveTask(channel_name);
  }
}

template <typename T>
void Task<T>::Execute(const std::shared_ptr<T>& val) {
  static auto it = name_ids_.begin();
  if (it == name_ids_.end()) {
    it = name_ids_.begin();
  }

  data::DataDispatcher<T>::Instance()->Dispatch(*it, val);
  running_count_++;
  ++it;
}

template <typename T>
void Task<T>::Finish() {
  std::lock_guard<std::mutex> lg(count_lock_);
  running_count_--;
}

template <typename T>
template <typename Rep, typename Period>
bool Task<T>::WaitFor(const std::chrono::duration<Rep, Period>& time) {
  auto wake_time = std::chrono::steady_clock::now() + time;
  while (std::chrono::steady_clock::now() < wake_time && !IsShutdown()) {
    {
      std::lock_guard<std::mutex> lg(count_lock_);
      if (running_count_ == 0) {
        return true;
      }
    }
    usleep(100000);
  }
  return false;
}

template <typename T>
bool Task<T>::Wait() {
  while (!IsShutdown()) {
    std::lock_guard<std::mutex> lg(count_lock_);
    if (running_count_ == 0) {
      return true;
    }
  }
  return false;
}

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_TASK_H_
