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

#ifndef CYBER_TIMER_TIMER_TASK_H_
#define CYBER_TIMER_TIMER_TASK_H_

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

using CallHandler = std::function<void()>;

class TimerTask {
 public:
  TimerTask(uint64_t id, uint64_t it, uint64_t ivl, CallHandler h, bool ons)
      : tid_(id), init_time_(it), interval_(ivl), handler_(h), oneshot_(ons) {}
  TimerTask() = default;

 private:
  enum STATS { INIT = 0, CANCELED, EXPIRED };
  STATS State() { return status_; }
  volatile STATS status_ = INIT;
  uint64_t tid_ = 0;

 public:
  uint64_t init_time_ = 0;
  uint64_t deadline_ = 0;
  uint64_t interval_ = 0;
  CallHandler handler_;
  uint64_t rest_rounds_ = 0;
  bool oneshot_ = true;
  uint64_t fire_count_ = 0;

 public:
  uint64_t Id() { return tid_; }

  void Fire(bool async);

  bool Cancel();

  bool IsCanceled() { return State() == CANCELED; }

  bool IsExpired() { return State() == EXPIRED; }
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMER_TASK_H_
