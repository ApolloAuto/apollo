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

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "cybertron/common/global_data.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/data/data_notifier.h"
#include "cybertron/init.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {

static const char* task_prefix = "/internal/task/";

template <typename Type, typename Ret>
struct TaskData {
  std::shared_ptr<Type> raw_data;
  std::promise<Ret> prom;
};

template <typename T, typename R, typename Derived>
class TaskBase {
 public:
  virtual ~TaskBase();
  void Stop();
  bool IsRunning() const;
  std::future<R> Execute(const std::shared_ptr<T>& val);
  std::shared_ptr<TaskData<T, R>> GetTaskData();

 protected:
  void RegisterCallback(
      std::function<void(const std::shared_ptr<TaskData<T, R>>&)>&& func);
  void RegisterCallback(std::function<void()>&& func);

 private:
  TaskBase(const std::string& name, const uint8_t& num_threads = 1);
  friend Derived;
  std::string name_;
  bool running_;
  uint32_t num_threads_;
  uint64_t task_id_;
  mutable std::mutex mutex_;
  std::list<std::shared_ptr<TaskData<T, R>>> data_list_;
};

template <typename T, typename R, typename Derived>
TaskBase<T, R, Derived>::TaskBase(const std::string& name,
                                  const uint8_t& num_threads)
    : num_threads_(num_threads), running_(true) {
  if (num_threads_ > scheduler::Scheduler::ProcessorNum()) {
    num_threads_ = scheduler::Scheduler::ProcessorNum();
  }
  name_ = task_prefix + name;
  task_id_ = common::GlobalData::RegisterChannel(name_);
}

template <typename T, typename R, typename Derived>
void TaskBase<T, R, Derived>::RegisterCallback(std::function<void()>&& func) {
  croutine::RoutineFactory factory = croutine::CreateRoutineFactory(func);
  for (int i = 0; i < num_threads_; ++i) {
    scheduler::Scheduler::Instance()->CreateTask(factory,
                                                 name_ + std::to_string(i));
  }
}

template <typename T = void, typename R = void>
class Task : public TaskBase<T, R, Task<T, R>> {
 public:
  template <typename Function>
  Task(const std::string& name, Function&& f, const uint8_t& num_threads = 1);
};

template <typename T, typename R>
template <typename Function>
Task<T, R>::Task(const std::string& name, Function&& f,
                 const uint8_t& num_threads)
    : TaskBase<T, R, Task<T, R>>(name, num_threads) {
  auto func = [ f = std::forward<Function&&>(f), this ]() {
    while (this->IsRunning()) {
      auto msg = this->GetTaskData();
      if (msg == nullptr) {
        auto routine = croutine::CRoutine::GetCurrentRoutine();
        routine->Sleep(1000);
        continue;
      }
      msg->prom.set_value(f(msg->raw_data));
    }
  };
  this->RegisterCallback(std::move(func));
}

template <typename T>
class Task<T, void> : public TaskBase<T, void, Task<T, void>> {
 private:
  typedef TaskBase<T, void, Task<T, void>> Base;

 public:
  template <typename Function>
  Task(const std::string& name, Function&& f, const uint8_t& num_threads = 1);
};

template <typename T>
template <typename Function>
Task<T, void>::Task(const std::string& name, Function&& f,
                    const uint8_t& num_threads)
    : TaskBase<T, void, Task<T, void>>(name, num_threads) {
  auto func = [ f = std::forward<Function&&>(f), this ]() {
    while (this->IsRunning()) {
      auto msg = this->GetTaskData();
      if (msg == nullptr) {
        auto routine = croutine::CRoutine::GetCurrentRoutine();
        routine->Sleep(1000);
        continue;
      }
      f(msg->raw_data);
      msg->prom.set_value();
    }
  };
  this->RegisterCallback(std::move(func));
}

template <typename T, typename R, typename Derived>
bool TaskBase<T, R, Derived>::IsRunning() const {
  std::lock_guard<std::mutex> lg(mutex_);
  return running_;
}

template <typename T, typename R, typename Derived>
void TaskBase<T, R, Derived>::Stop() {
  std::lock_guard<std::mutex> lg(mutex_);
  data_list_.clear();
  running_ = false;
}

template <typename T, typename R, typename Derived>
TaskBase<T, R, Derived>::~TaskBase() {
  Stop();
  for (int i = 0; i < num_threads_; ++i) {
    scheduler::Scheduler::Instance()->RemoveTask(name_ + std::to_string(i));
  }
}

template <typename T, typename R, typename Derived>
std::future<R> TaskBase<T, R, Derived>::Execute(const std::shared_ptr<T>& val) {
  auto task = std::make_shared<TaskData<T, R>>();
  task->raw_data = val;
  {
    std::lock_guard<std::mutex> lg(mutex_);
    data_list_.emplace_back(task);
  }
  // data::DataNotifier::Instance()->Notify(task_id_);
  return task->prom.get_future();
}

template <typename T, typename R, typename Derived>
std::shared_ptr<TaskData<T, R>> TaskBase<T, R, Derived>::GetTaskData() {
  std::lock_guard<std::mutex> lg(mutex_);
  if (!running_) {
    return nullptr;
  }

  if (data_list_.empty()) {
    return nullptr;
  }
  auto task = data_list_.front();
  data_list_.pop_front();
  return task;
}

template <>
template <typename Function>
Task<void, void>::Task(const std::string& name, Function&& f,
                       const uint8_t& num_threads)
    : TaskBase<void, void, Task<void, void>>(name, num_threads) {
  auto data = std::make_shared<TaskData<void, void>>();
  auto func = [ f = std::forward<Function&&>(f), data ]() {
    f();
    data->prom.set_value();
  };
  RegisterCallback(std::move(func));
}

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_SCHEDULER_TASK_H_
