/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

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

#include "cyber/time/rate.h"

#include <chrono>
#include <thread>

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {

Rate::Rate(double frequency)
    : start_(Time::MonoTime()),
      expected_cycle_time_(1.0 / frequency),
      actual_cycle_time_(0.0) {}

Rate::Rate(uint64_t nanoseconds)
    : start_(Time::MonoTime()),
      expected_cycle_time_(static_cast<int64_t>(nanoseconds)),
      actual_cycle_time_(0.0) {}

Rate::Rate(const Duration& d)
    : start_(Time::MonoTime()),
      expected_cycle_time_(d),
      actual_cycle_time_(0.0) {}

void Rate::Sleep() {
  const Time expected_end = start_ + expected_cycle_time_;
  const Time actual_end = Time::MonoTime();
  const Duration sleep_time = expected_end - actual_end;

  actual_cycle_time_ = actual_end - start_;

  // Advance from the previous deadline so wake-up delays do not accumulate.
  start_ = expected_end;

  if (sleep_time < Duration(0.0)) {
    AWARN << "Rate cycle overrun";
    // Resynchronize if the loop is more than one full cycle behind.
    if (actual_end > expected_end + expected_cycle_time_) {
      start_ = actual_end;
    }
    return;
  }

  // Use the same clock as Time::MonoTime(). Time::SleepUntil() expects a
  // wall-clock timestamp, so it cannot be used with this monotonic deadline.
  const auto deadline = std::chrono::steady_clock::time_point(
      std::chrono::nanoseconds(expected_end.ToNanosecond()));
  std::this_thread::sleep_until(deadline);
}

void Rate::Reset() { start_ = Time::MonoTime(); }

Duration Rate::CycleTime() const { return actual_cycle_time_; }

}  // namespace cyber
}  // namespace apollo
