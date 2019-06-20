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

#ifndef CYBER_TIMER_TIMER_TASK_H_
#define CYBER_TIMER_TIMER_TASK_H_

#include <functional>

namespace apollo {
namespace cyber {

class TimerBucket;

struct TimerTask {
  explicit TimerTask(uint64_t timer_id) : timer_id_(timer_id) {}
  uint64_t timer_id_ = 0;
  std::function<void()> callback;
  uint64_t interval_ms = 0;
  uint64_t remainder_interval_ms = 0;
  uint64_t next_fire_duration_ms = 0;
  int64_t accumulated_error_ns_ = 0;
  uint64_t last_execute_time_ns_ = 0;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMER_TASK_H_
