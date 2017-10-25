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

#ifndef MODULES_MONITOR_COMMON_COMMON_DEFS_H_
#define MODULES_MONITOR_COMMON_COMMON_DEFS_H_

#include <utility>

#ifndef NULL
#define NULL 0
#endif

#ifdef __GNUC__
#define UNUSED(x) UNUSED_##x __attribute__((__unused__))
#else
#define UNUSED(x) UNUSED_##x
#endif

#define DO_ONCE(args...)                   \
  do {                                     \
    static int __##__FILE__##__LINE__ = 0; \
    if (__##__FILE__##__LINE__++ < 1) {    \
      args;                                \
    }                                      \
  } while (0)

#ifndef PARAM_IN
#define PARAM_IN
#endif

#ifndef PARAM_OUT
#define PARAM_OUT
#endif

#ifndef MUST_USE
/// Indicating the caller must check the return value of the annotated function.
#define MUST_USE
#endif

#ifdef _DEBUG
#define DBG_VAR(t, v) t v
#define DBG_ONLY(args...) \
  do {                    \
    args;                 \
  } while (0)
#else
#define DBG_VAR(t, v)
#define DBG_ONLY(args...)
#endif

/// Convenient helper macro to create a setter member function which returns the
/// old value.
#define MEMBER_SET_AND_RETURN(T, var) \
  T set_##var(T val) {                \
    T old_val = std::move(var);       \
    var = std::move(val);             \
    return old_val;                   \
  }

#endif  // MODULES_MONITOR_COMMON_COMMON_DEFS_H_
