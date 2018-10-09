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
#pragma once

#include <string>

#include "modules/perception/lib/utils/timer.h"

// Usage:
// Firstly, you can define PERCEPTION_DISABLE_PERF to disable all perf infos
// in compiling time.
//
// Using PERCEPTION_PERF_FUNCTION to perf the entire function time elapsed.
// as following code:
//      void MyFunc() {
//          PERCEPTION_PERF_FUNCION();
//          // do somethings.
//      }
//  log would output:
//  >>>>>>>>>>>>>>>>>>
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER MyFunc elapsed time: 100 ms
//  >>>>>>>>>>>>>>>>>>
//
//  Using PERCEPTION_PERF_BLOCK to perf code block time elapsed.
//  as following code:
//      void MyFunc() {
//          // xxx1
//          PERCEPTION_PERF_BLOCK_START();
//
//          // do xx2
//
//          PERCEPTION_PERF_BLOCK_END("xx2");
//
//          // do xx3
//
//          PERCEPTION_PERF_BLOCK_END("xx3");
//      }
//
//  log would output:
//  >>>>>>>>>>>>>>>
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER xx2 elapsed time: 100 ms
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER xx3 elapsed time: 200 ms
//  >>>>>>>>>>>>>>>

namespace apollo {
namespace perception {
namespace lib {

inline std::string get_full_name(const std::string& full_name) {
  size_t end = full_name.find("(");

  if (end == std::string::npos) {
    return full_name;
  }

  std::string new_str = full_name.substr(0, end);
  size_t start = new_str.rfind(" ");

  if (start == std::string::npos) {
    return full_name;
  }

  return new_str.substr(start + 1);
}

inline std::string get_full_name(const std::string& full_name,
                                 const std::string& indicator) {
  return indicator + "_" + get_full_name(full_name);
}

}  // namespace lib
}  // namespace perception
}  // namespace apollo

#ifdef PERCEPTION_DISABLE_PERF

// disable macros.
#define PERCEPTION_PERF_FUNCTION()

#define PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(indicator)

#define PERCEPTION_PERF_BLOCK_START()

#define PERCEPTION_PERF_BLOCK_END(msg)

#define PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(indicator, msg)

#else

#define PERCEPTION_PERF_FUNCTION()                       \
  apollo::perception::lib::TimerWrapper _timer_wrapper_( \
      apollo::perception::lib::get_full_name(__PRETTY_FUNCTION__))

#define PERCEPTION_PERF_FUNCTION_WITH_INDICATOR(indicator) \
  apollo::perception::lib::TimerWrapper _timer_wrapper_(   \
      apollo::perception::lib::get_full_name(__PRETTY_FUNCTION__, indicator))

#define PERCEPTION_PERF_BLOCK_START()     \
  apollo::perception::lib::Timer _timer_; \
  _timer_.Start()

#define PERCEPTION_PERF_BLOCK_END(msg) _timer_.End(msg)

#define PERCEPTION_PERF_BLOCK_END_WITH_INDICATOR(indicator, msg) \
  _timer_.End(indicator + "_" + msg)

#endif  // PERCEPTION_DISABLE_PERF
