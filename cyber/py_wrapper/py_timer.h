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

#ifndef CYBER_PY_WRAPPER_PY_TIMER_H_
#define CYBER_PY_WRAPPER_PY_TIMER_H_

#include <unistd.h>
#include <functional>
#include <memory>

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/timer/timer.h"

namespace apollo {
namespace cyber {

class PyTimer {
 public:
  PyTimer() { timer_ = std::make_shared<Timer>(); }

  PyTimer(uint32_t period, void (*func)(), bool oneshot) {
    std::function<void()> bound_f = std::bind(func);
    timer_ = std::make_shared<Timer>(period, bound_f, oneshot);
  }

  void start() { timer_->Start(); }

  void stop() { timer_->Stop(); }

  void set_option(uint32_t period, void (*func)(), bool oneshot) {
    std::function<void()> bound_f = std::bind(func);
    TimerOption time_opt;
    time_opt.period = period;
    time_opt.callback = bound_f;
    time_opt.oneshot = oneshot;
    timer_->SetTimerOption(time_opt);
  }

 private:
  std::shared_ptr<Timer> timer_ = nullptr;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_PY_WRAPPER_PY_TIMER_H_
