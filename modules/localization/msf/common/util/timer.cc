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

#include "modules/localization/msf/common/util/timer.h"
#include "modules/common/log.h"

namespace apollo {
namespace localization {
namespace msf {

Timer::Timer() {}

void Timer::Start() {
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

void Timer::End(const char* title) {
  end_time_ = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration dt = end_time_ - start_time_;
  if (title) {
    AINFO << title << " Elapsed time: " << dt.seconds() << "s "
          << (dt.total_milliseconds() - dt.seconds() * 1000) << "ms";
  } else {
    AINFO << " Elapsed time: " << dt.seconds() << "s "
          << (dt.total_milliseconds() - dt.seconds() * 1000) << "ms";
  }
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

TimeAccumulator::TimeAccumulator() {}

void TimeAccumulator::Start() {
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

void TimeAccumulator::End(const char* title) {
  boost::posix_time::ptime end_time =
      boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration dt = end_time - start_time_;
  duration_ += dt;
  if (title) {
    AINFO << title << " Elapsed time: " << dt.seconds() << "s "
          << (dt.total_milliseconds() - dt.seconds() * 1000) << "ms";
  }
  start_time_ = boost::posix_time::microsec_clock::local_time();
}

void TimeAccumulator::Clear() {
  duration_ = boost::posix_time::time_duration();
}

void TimeAccumulator::GetDuration(const char* title) {
  if (title) {
    AINFO << title << " Total elapsed time: " << duration_.seconds() << "s "
          << (duration_.total_milliseconds() - duration_.seconds() * 1000)
          << "ms";
  } else {
    AINFO << " Total elapsed time: " << duration_.seconds() << "s "
          << (duration_.total_milliseconds() - duration_.seconds() * 1000)
          << "ms";
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
