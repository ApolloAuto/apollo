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

#include "modules/perception/lib/base/timer.h"
#include "modules/common/log.h"

namespace apollo {
namespace perception {

using std::string;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

void Timer::start() {
  _start_time = std::chrono::system_clock::now();
}

uint64_t Timer::end(const string& msg) {
  _end_time = std::chrono::system_clock::now();
  uint64_t elapsed_time =
      duration_cast<milliseconds>(_end_time - _start_time).count();

  ADEBUG << "TIMER " << msg << " elapsed_time: " << elapsed_time << " ms";

  // start new timer.
  _start_time = _end_time;
  return elapsed_time;
}

}  // namespace perception
}  // namespace apollo
