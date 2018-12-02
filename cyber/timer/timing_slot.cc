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

#include "cyber/timer/timing_slot.h"

#include "cyber/common/log.h"
#include "cyber/timer/timer_task.h"

namespace apollo {
namespace cyber {

void TimingSlot::AddTask(const std::shared_ptr<TimerTask>& task) {
  tasks_.emplace(task->Id(), task);
}

void TimingSlot::EnumTaskList(
    uint64_t deadline, bool async, BoundedQueue<HandlePackage>* hander_queue,
    BoundedQueue<std::shared_ptr<TimerTask>>* rep_queue) {
  for (auto it = tasks_.begin(); it != tasks_.end();) {
    auto task = it->second;  // *it;
    auto del_it = it;
    it++;
    // std::cout << "judge: task->" << task->deadline << " : " << deadline;
    if (task->deadline_ <= deadline) {
      if (task->rest_rounds_ == 0) {
        if (async) {
          HandlePackage hp;
          hp.handle = task->handler_;
          hp.id = task->Id();
          if (!hander_queue->Enqueue(hp)) {
            AERROR << "hander queue is full";
          }
        } else {
          task->Fire(false);
        }

        if (!task->oneshot_) {  // repeat timer,push back
          task->fire_count_++;
          rep_queue->Enqueue(task);
        }
        tasks_.erase(del_it);

      } else {
        AERROR << "task deadline overflow...";
      }
    } else {  // no expired, -- rounds
      task->rest_rounds_--;
    }
  }
}
}  // namespace cyber
}  // namespace apollo
