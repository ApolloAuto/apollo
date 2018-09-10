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

#include "cybertron/timer/timer_task.h"

#include "cybertron/common/log.h"
#include "cybertron/time/time.h"
#include "cybertron/timer/timing_wheel.h"

namespace apollo {
namespace cybertron {

void TimerTask::Fire(bool async) {
  if (status_ != INIT) {
    return;
  }
  if (oneshot_)  // not repeat. so always on ready
    status_ = EXPIRED;
  if (async) {
    TimingWheel::Workers().Enqueue(handler_);
  } else {
    handler_();
  }
}

bool TimerTask::Cancel() {
  if (State() != INIT) {
    return false;
  }
  status_ = CANCELED;
  return true;
}
}  // namespace cybertron
}  // namespace apollo
