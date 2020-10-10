/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_TIMER_TIMER_BUCKET_H_
#define CYBER_TIMER_TIMER_BUCKET_H_

#include <list>
#include <memory>
#include <mutex>

#include "cyber/timer/timer_task.h"

namespace apollo {
namespace cyber {

class TimerBucket {
 public:
  void AddTask(const std::shared_ptr<TimerTask>& task) {
    std::lock_guard<std::mutex> lock(mutex_);
    task_list_.push_back(task);
  }

  std::mutex& mutex() { return mutex_; }
  std::list<std::weak_ptr<TimerTask>>& task_list() { return task_list_; }

 private:
  std::mutex mutex_;
  std::list<std::weak_ptr<TimerTask>> task_list_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMER_BUCKET_H_
