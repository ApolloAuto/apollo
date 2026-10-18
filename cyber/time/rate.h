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

#ifndef CYBER_TIME_RATE_H_
#define CYBER_TIME_RATE_H_

#include "cyber/time/duration.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {

/**
 * @class Rate
 * @brief Regulate a loop using monotonic time.
 * System clock adjustments do not affect the cycle duration or sleep deadline.
 */
class Rate {
 public:
  explicit Rate(double frequency);
  explicit Rate(uint64_t nanoseconds);
  explicit Rate(const Duration&);

  /**
   * @brief Sleep until the next monotonic cycle deadline.
   * Return immediately if the cycle has already overrun its expected duration.
   */
  void Sleep();

  /**
   * @brief Start a new cycle at the current monotonic time.
   */
  void Reset();

  /**
   * @brief Get the elapsed cycle time measured by the last Sleep(), before
   * waiting for the deadline.
   */
  Duration CycleTime() const;

  /**
   * @brief Get the configured cycle duration.
   */
  Duration ExpectedCycleTime() const { return expected_cycle_time_; }

 private:
  Time start_;  // Monotonic timestamp, not a wall-clock timestamp.
  Duration expected_cycle_time_;
  Duration actual_cycle_time_;
};

}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TIME_RATE_H_
