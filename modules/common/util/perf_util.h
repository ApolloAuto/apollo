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

#include "absl/strings/str_cat.h"

#include "cyber/common/macros.h"
#include "modules/common/time/timer.h"

#if defined(__GNUC__) || defined(__GNUG__)
#define AFUNC __PRETTY_FUNCTION__
#elif defined(__clang__)
#define AFUNC __PRETTY_FUNCTION__
#else
#define AFUNC __func__
#endif

// How to Use:
// 1)  Use PERF_FUNCTION to compute time cost of function execution as follows:
//      void MyFunc() {
//          PERF_FUNCION();
//          // do somethings.
//      }
//  Console log:
//  >>>>>>>>>>>>>>>>>>
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER MyFunc elapsed time: 100 ms
//  >>>>>>>>>>>>>>>>>>
//
//  2) Use PERF_BLOCK_START/END to compute time cost of block execution.
//      void MyFunc() {
//          // xxx1
//          PERF_BLOCK_START();
//
//          // do xx2
//
//          PERF_BLOCK_END("xx2");
//
//          // do xx3
//
//          PERF_BLOCK_END("xx3");
//      }
//
//  Console log:
//  >>>>>>>>>>>>>>>
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER xx2 elapsed time: 100 ms
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER xx3 elapsed time: 200 ms
//  >>>>>>>>>>>>>>>
namespace apollo {
namespace common {
namespace util {

std::string function_signature(const std::string& func_name,
                               const std::string& indicator = "");

}  // namespace util
}  // namespace common
}  // namespace apollo

#if defined(ENABLE_PERF)
#define PERF_FUNCTION()                               \
  apollo::common::time::TimerWrapper _timer_wrapper_( \
      apollo::common::util::function_signature(AFUNC))
#define PERF_FUNCTION_WITH_NAME(func_name) \
  apollo::common::time::TimerWrapper _timer_wrapper_(func_name)
#define PERF_FUNCTION_WITH_INDICATOR(indicator)       \
  apollo::common::time::TimerWrapper _timer_wrapper_( \
      apollo::common::util::function_signature(AFUNC, indicator))
#define PERF_BLOCK_START()             \
  apollo::common::time::Timer _timer_; \
  _timer_.Start()
#define PERF_BLOCK_END(msg) _timer_.End(msg)
#define PERF_BLOCK_END_WITH_INDICATOR(indicator, msg) \
  _timer_.End(absl::StrCat(indicator, "_", msg))
#else
#define PERF_FUNCTION()
#define PERF_FUNCTION_WITH_NAME(func_name) UNUSED(func_name);
#define PERF_FUNCTION_WITH_INDICATOR(indicator) UNUSED(indicator);
#define PERF_BLOCK_START()
#define PERF_BLOCK_END(msg) UNUSED(msg);
#define PERF_BLOCK_END_WITH_INDICATOR(indicator, msg) \
  {                                                   \
    UNUSED(indicator);                                \
    UNUSED(msg);                                      \
  }
#endif  // ENABLE_PERF
