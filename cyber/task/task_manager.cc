/**************Scheduler::****************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Vesched_infoon 2.0 (the "License");
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

#include "cyber/task/task_manager.h"

#include "cyber/common/global_data.h"
#include "cyber/croutine/croutine.h"
#include "cyber/croutine/routine_factory.h"
#include "cyber/scheduler/scheduler_factory.h"

namespace apollo {
namespace cyber {

using apollo::cyber::common::GlobalData;
static const char* const task_prefix = "/internal/task";

TaskManager::TaskManager()
    : task_queue_size_(1000),
      task_queue_(new base::BoundedQueue<std::function<void()>>()) {
  if (!task_queue_->Init(task_queue_size_, new base::BlockWaitStrategy())) {
    AERROR << "Task queue init failed";
    throw std::runtime_error("Task queue init failed");
  }
  auto func = [this]() {
    while (!stop_) {
      std::function<void()> task;
      if (!task_queue_->Dequeue(&task)) {
        auto routine = croutine::CRoutine::GetCurrentRoutine();
        routine->HangUp();
        continue;
      }
      task();
    }
  };

  num_threads_ = scheduler::Instance()->TaskPoolSize();
  auto factory = croutine::CreateRoutineFactory(std::move(func));
  tasks_.reserve(num_threads_);
  for (uint32_t i = 0; i < num_threads_; i++) {
    auto task_name = task_prefix + std::to_string(i);
    tasks_.push_back(common::GlobalData::RegisterTaskName(task_name));
    if (!scheduler::Instance()->CreateTask(factory, task_name)) {
      AERROR << "CreateTask failed:" << task_name;
    }
  }
}

TaskManager::~TaskManager() { Shutdown(); }

void TaskManager::Shutdown() {
  if (stop_.exchange(true)) {
    return;
  }
  for (uint32_t i = 0; i < num_threads_; i++) {
    scheduler::Instance()->RemoveTask(task_prefix + std::to_string(i));
  }
}

}  // namespace cyber
}  // namespace apollo
