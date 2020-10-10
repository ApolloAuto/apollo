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

#ifndef CYBER_TASK_TASK_MANAGER_H_
#define CYBER_TASK_TASK_MANAGER_H_

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "cyber/base/bounded_queue.h"
#include "cyber/scheduler/scheduler_factory.h"

namespace apollo {
namespace cyber {

class TaskManager {
 public:
  virtual ~TaskManager();

  void Shutdown();

  template <typename F, typename... Args>
  auto Enqueue(F&& func, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type> {
    using return_type = typename std::result_of<F(Args...)>::type;
    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(func), std::forward<Args>(args)...));
    if (!stop_.load()) {
      task_queue_->Enqueue([task]() { (*task)(); });
      for (auto& task : tasks_) {
        scheduler::Instance()->NotifyTask(task);
      }
    }
    std::future<return_type> res(task->get_future());
    return res;
  }

 private:
  uint32_t num_threads_ = 0;
  uint32_t task_queue_size_ = 1000;
  std::atomic<bool> stop_ = {false};
  std::vector<uint64_t> tasks_;
  std::shared_ptr<base::BoundedQueue<std::function<void()>>> task_queue_;
  DECLARE_SINGLETON(TaskManager);
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TASK_TASK_MANAGER_H_
