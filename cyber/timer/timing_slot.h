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

#ifndef CYBER_TIMER_TIMING_SLOT_H_
#define CYBER_TIMER_TIMING_SLOT_H_

#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cyber/base/bounded_queue.h"

namespace apollo {
namespace cyber {

using apollo::cyber::base::BoundedQueue;
using CallHandler = std::function<void()>;

class TimerTask;

struct HandlePackage {
  uint64_t id;
  CallHandler handle;
};

class TimingSlot {
 private:
  //  no needs for multi-thread
  std::unordered_map<uint64_t, std::shared_ptr<TimerTask>> tasks_;

 public:
  TimingSlot() = default;
  void AddTask(const std::shared_ptr<TimerTask>& task);
  void RemoveTask(uint64_t id) { tasks_.erase(id); }

  void EnumTaskList(uint64_t deadline, bool async,
                    BoundedQueue<HandlePackage>* hander_queue,
                    BoundedQueue<std::shared_ptr<TimerTask>>* rep_list);
};  // TimeSlot end

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMING_SLOT_H_
