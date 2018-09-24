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

#ifndef CYBERTRON_TASK_TASK_MANAGER_H_
#define CYBERTRON_TASK_TASK_MANAGER_H_

#include <memory>
#include <string>

#include "cybertron/base/bounded_queue.h"
#include "cybertron/scheduler/scheduler.h"

namespace apollo {
namespace cybertron {

class TaskManager {
 public:
  virtual ~TaskManager();

  template <typename F, typename... Args>
  auto Enqueue(F&& func, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type> {
    using return_type = typename std::result_of<F(Args...)>::type;
    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(func), std::forward<Args>(args)...));
    task_queue_->Enqueue([task]() { (*task)(); });
    for (auto& task : tasks_) {
      scheduler::Scheduler::Instance()->NotifyTask(task);
    }
    std::future<return_type> res(task->get_future());
    return res;
  }

 private:
  uint32_t num_threads_;
  uint32_t task_queue_size_ = 1000;
  std::vector<uint64_t> tasks_;
  std::shared_ptr<base::BoundedQueue<std::function<void()>>> task_queue_;
  DECLARE_SINGLETON(TaskManager);
};

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_TASK_TASK_MANAGER_H_
