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

#include "cybertron/task/task_manager.h"

#include "cybertron/task/task.h"
#include "cybertron/croutine/croutine.h"
#include "cybertron/croutine/routine_factory.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {

static const char* const task_prefix = "/internal/task";

TaskManager::TaskManager()
    : task_queue_size_(1000),
      task_queue_(new base::BoundedQueue<std::function<void()>>()) {
  task_queue_->Init(task_queue_size_, new base::BlockWaitStrategy());
  auto func = [task_queue = this->task_queue_]() {
    while (!cybertron::IsShutdown()) {
      std::function<void()> task;
      task_queue->Dequeue(&task);
      if (task == nullptr) {
        auto routine = croutine::CRoutine::GetCurrentRoutine();
        routine->HangUp();
        continue;
      }
      task();
    }
  };

  auto factory = croutine::CreateRoutineFactory(std::move(func));
  num_threads_ = scheduler::Scheduler::ProcessorNum();
  tasks_.reserve(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    auto task_name = task_prefix + std::to_string(i);
    tasks_.push_back(GlobalData::RegisterTaskName(task_name));
    scheduler::Scheduler::Instance()->CreateTask(factory, task_name);
  }
}

TaskManager::~TaskManager() {
  for (int i = 0; i < num_threads_; i++) {
    scheduler::Scheduler::Instance()->RemoveTask(task_prefix +
                                                 std::to_string(i));
  }
}

}  // namespace cybertron
}  // namespace apollo
