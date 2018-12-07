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

#ifndef CYBER_TIMER_TIMER_H_
#define CYBER_TIMER_TIMER_H_

#include <atomic>
#include <memory>

#include "cyber/timer/timer_manager.h"

namespace apollo {
namespace cyber {

struct TimerOption {
  uint32_t period;                 // The period of the timer, unit is ms
  std::function<void()> callback;  // The tasks that the timer needs to perform
  bool oneshot;  // True: perform the callback only after the first timing cycle
                 // False: perform the callback every timed period
};

class Timer {
 public:
  Timer();

  /**
   * @brief Construct a new Timer object
   *
   * @param opt Timer option
   */
  explicit Timer(TimerOption opt);

  /**
   * @brief Construct a new Timer object
   *
   * @param period The period of the timer, unit is ms
   * @param callback The tasks that the timer needs to perform
   * @param oneshot True: perform the callback only after the first timing cycle
   *                False: perform the callback every timed period
   */
  Timer(uint32_t period, std::function<void()> callback, bool oneshot);
  ~Timer();

  /**
   * @brief Set the Timer Option object
   *
   * @param opt The timer option will be set
   */
  void SetTimerOption(TimerOption opt);

  /**
   * @brief Start the timer
   *
   */
  void Start();

  /**
   * @brief Stop the timer
   *
   */
  void Stop();

 private:
  TimerOption timer_opt_;
  TimerManager* tm_ = nullptr;
  uint64_t timer_id_ = 0;
  std::atomic<bool> started_ = {false};
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMER_H_
