/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "periodic_task.h"

#include <thread>

namespace apollo {
namespace monitor {
namespace sysmon {

// @fixme: we don't check task over-run -- i.e., runs longer than the interval.
void PeriodicTask::run() {
  do {
    std::chrono::steady_clock::time_point next_tp
        = std::chrono::steady_clock::now() + _interval;

    // @todo: print timestamp
    DBG_ONLY(
      ADEBUG << "to run task for the " << ++_run_cnt << "th time (task=%p)";
    )

    do_task();

    if (_stop_flag.load()) {
      break;
    }
    std::this_thread::sleep_until(next_tp);
  } while (!_stop_flag.load());
}

}  // namespace sysmon
}  // namespace platform
}  // namespace apollo
