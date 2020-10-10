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

#ifndef CYBER_TASK_TASK_H_
#define CYBER_TASK_TASK_H_

#include <future>
#include <utility>

#include "cyber/task/task_manager.h"

namespace apollo {
namespace cyber {

using apollo::cyber::common::GlobalData;

template <typename F, typename... Args>
static auto Async(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type> {
  return GlobalData::Instance()->IsRealityMode()
             ? TaskManager::Instance()->Enqueue(std::forward<F>(f),
                                                std::forward<Args>(args)...)
             : std::async(
                   std::launch::async,
                   std::bind(std::forward<F>(f), std::forward<Args>(args)...));
}

static inline void Yield() {
  if (croutine::CRoutine::GetCurrentRoutine()) {
    croutine::CRoutine::Yield();
  } else {
    std::this_thread::yield();
  }
}

template <typename Rep, typename Period>
static void SleepFor(const std::chrono::duration<Rep, Period>& sleep_duration) {
  auto routine = croutine::CRoutine::GetCurrentRoutine();
  if (routine == nullptr) {
    std::this_thread::sleep_for(sleep_duration);
  } else {
    routine->Sleep(sleep_duration);
  }
}

static inline void USleep(useconds_t usec) {
  auto routine = croutine::CRoutine::GetCurrentRoutine();
  if (routine == nullptr) {
    std::this_thread::sleep_for(std::chrono::microseconds{usec});
  } else {
    routine->Sleep(croutine::Duration(usec));
  }
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TASK_TASK_H_
