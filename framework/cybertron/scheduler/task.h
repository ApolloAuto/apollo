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
#include "cybertron/data/data_dispatcher.h"
#include "cybertron/init.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {

static const char* task_prefix = "/internal/task/";

template <typename Type, typename Ret>
class TaskData {
 public:
  std::shared_ptr<Type> raw_data;
  std::promise<Ret> prom;
};

template <typename T, typename R, typename Derived>
class TaskBase {
 public:
  ~TaskBase();
  std::future<R> Execute(const std::shared_ptr<T>& val);

 protected:
  void RegisterCallback(
      std::function<void(const std::shared_ptr<TaskData<T, R>>&)>&& func);
  void RegisterCallback(std::function<void()>&& func);
  uint32_t num_threads_;
  std::string name_;
  std::vector<uint64_t> name_ids_;

 private:
  TaskBase(const std::string& name, const uint8_t& num_threads = 1);
  friend Derived;
};

template <typename T, typename R, typename Derived>
TaskBase<T, R, Derived>::TaskBase(const std::string& name,
                                  const uint8_t& num_threads)
    : num_threads_(num_threads) {
  name_ = task_prefix + name;
  name_ids_.reserve(num_threads);
}

template <typename T, typename R, typename Derived>
void TaskBase<T, R, Derived>::RegisterCallback(
    std::function<void(const std::shared_ptr<TaskData<T, R>>&)>&& func) {
  for (int i = 0; i < num_threads_; ++i) {
    auto channel_name = name_ + std::to_string(i);
    auto name_id = common::GlobalData::RegisterChannel(channel_name);
    name_ids_.push_back(std::move(name_id));
    auto dv = std::make_shared<data::DataVisitor<TaskData<T, R>>>(name_id, 1);
    croutine::RoutineFactory factory =
        croutine::CreateRoutineFactory<TaskData<T, R>>(func, dv);
    scheduler::Scheduler::Instance()->CreateTask(factory, channel_name);
  }
}

template <typename T, typename R, typename Derived>
void TaskBase<T, R, Derived>::RegisterCallback(std::function<void()>&& func) {
  for (int i = 0; i < num_threads_; ++i) {
    auto channel_name = name_ + std::to_string(i);
    auto name_id = common::GlobalData::RegisterChannel(channel_name);
    name_ids_.push_back(std::move(name_id));
    croutine::RoutineFactory factory = croutine::CreateRoutineFactory(func);
    scheduler::Scheduler::Instance()->CreateTask(factory, channel_name);
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
  auto func = [f = std::forward<Function&&>(f)](
      const std::shared_ptr<TaskData<T, R>>& msg) {
    msg->prom.set_value(f(msg->raw_data));
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
  auto func = [f = std::forward<Function&&>(f)](
      const std::shared_ptr<TaskData<T, void>>& msg) {
    f(msg->raw_data);
    msg->prom.set_value();
  };
  this->RegisterCallback(std::move(func));
}

template <typename T, typename R, typename Derived>
TaskBase<T, R, Derived>::~TaskBase() {
  for (int i = 0; i < num_threads_; ++i) {
    auto channel_name = task_prefix + name_ + std::to_string(i);
    scheduler::Scheduler::Instance()->RemoveTask(channel_name);
  }
}

template <typename T, typename R, typename Derived>
std::future<R> TaskBase<T, R, Derived>::Execute(const std::shared_ptr<T>& val) {
  static auto it = name_ids_.begin();
  if (it == name_ids_.end()) {
    it = name_ids_.begin();
  }
  auto task = std::make_shared<TaskData<T, R>>();
  task->raw_data = val;
  data::DataDispatcher<TaskData<T, R>>::Instance()->Dispatch(*it, task);
  ++it;
  return task->prom.get_future();
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
