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

#ifndef CYBER_STATE_H_
#define CYBER_STATE_H_

#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <thread>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {

enum State : std::uint8_t {
  STATE_UNINITIALIZED = 0,
  STATE_INITIALIZED,
  STATE_SHUTTING_DOWN,
  STATE_SHUTDOWN,
};

State GetState();
void SetState(const State& state);

inline bool OK() { return GetState() == STATE_INITIALIZED; }

inline bool IsShutdown() {
  return GetState() == STATE_SHUTTING_DOWN || GetState() == STATE_SHUTDOWN;
}

inline void WaitForShutdown() {
  while (!IsShutdown()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

inline void AsyncShutdown() {
  pid_t pid = getpid();
  if (kill(pid, SIGINT) != 0) {
    AERROR << strerror(errno);
  }
}

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_STATE_H_
