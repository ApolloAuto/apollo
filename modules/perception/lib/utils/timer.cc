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

#include <sys/time.h>

#include "cyber/common/log.h"
#include "modules/perception/lib/utils/timer.h"

namespace apollo {
namespace perception {
namespace lib {

using std::string;

void Timer::Start() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  start_time_ = tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

uint64_t Timer::End(const string &msg) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  end_time_ = tv.tv_sec * 1000 + tv.tv_usec / 1000;
  uint64_t elapsed_time = end_time_ - start_time_;

  ADEBUG << "TIMER " << msg << " elapsed_time: " << elapsed_time << " ms";

  // start new timer.
  start_time_ = end_time_;
  return elapsed_time;
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo
