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

#ifndef CYBER_TIMER_TIMER_H_
#define CYBER_TIMER_TIMER_H_

#include <atomic>
#include <memory>

#include "cyber/timer/timing_wheel.h"

namespace apollo {
namespace cyber {

/**
 * @brief The options of timer
 *
 */
struct TimerOption {
  /**
   * @brief Construct a new Timer Option object
   *
   * @param period The period of the timer, unit is ms
   * @param callback The task that the timer needs to perform
   * @param oneshot Oneshot or period
   */
  TimerOption(uint32_t period, std::function<void()> callback, bool oneshot)
      : period(period), callback(callback), oneshot(oneshot) {}

  /**
   * @brief Default constructor for initializer list
   *
   */
  TimerOption() : period(), callback(), oneshot() {}

  /**
   * @brief The period of the timer, unit is ms
   * max: 512 * 64
   * min: 1
   */
  uint32_t period = 0;

  /**The task that the timer needs to perform*/
  std::function<void()> callback;

  /**
   * True: perform the callback only after the first timing cycle
   * False: perform the callback every timed period
   */
  bool oneshot;
};

/**
 * @class Timer
 * @brief Used to perform oneshot or periodic timing tasks
 *
 */
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
  bool InitTimerTask();
  uint64_t timer_id_;
  TimerOption timer_opt_;
  TimingWheel* timing_wheel_ = nullptr;
  std::shared_ptr<TimerTask> task_;
  std::atomic<bool> started_ = {false};
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIMER_TIMER_H_
