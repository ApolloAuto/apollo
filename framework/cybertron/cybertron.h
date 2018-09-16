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

#ifndef CYBERTRON_CYBERTRON_H_
#define CYBERTRON_CYBERTRON_H_

#include <memory>
#include <string>
#include <utility>

#include "cybertron/common/log.h"
#include "cybertron/component/component.h"
#include "cybertron/init.h"
#include "cybertron/node/node.h"
#include "cybertron/scheduler/task.h"
#include "cybertron/time/time.h"
#include "cybertron/timer/timer.h"

namespace apollo {
namespace cybertron {

std::unique_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space = "");

template <typename Function>
std::unique_ptr<Task<>> CreateTask(const std::string& name, Function&& f,
                                   const uint8_t num_threads = 1) {
  if (!OK()) {
    return nullptr;
  }
  std::unique_ptr<Task<>> task(
      new Task<>(name, std::forward<Function>(f), num_threads));
  return std::move(task);
}

template <typename T, typename Function>
std::unique_ptr<Task<T>> CreateTask(const std::string& name, Function&& f,
                                    const uint8_t& num_threads = 1) {
  if (!OK()) {
    return nullptr;
  }
  std::unique_ptr<Task<T>> task(
      new Task<T>(name, std::forward<Function>(f), num_threads));
  return std::move(task);
}

inline static void Yield() {
  if (croutine::CRoutine::GetCurrentRoutine()) {
    croutine::CRoutine::Yield();
  } else {
    std::this_thread::yield();
  }
}

inline static void USleep(useconds_t usec) {
  auto routine = croutine::CRoutine::GetCurrentRoutine();
  if (routine == nullptr) {
    std::this_thread::sleep_for(std::chrono::microseconds{usec});
  } else {
    routine->Sleep(usec);
  }
}

}  // namespace cybertron
}  // namespace apollo

#endif  // CYBERTRON_CYBERTRON_H_
