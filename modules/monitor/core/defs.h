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

#ifndef MODULES_MONITOR_SYSMON_CORE_DEFS_H_
#define MODULES_MONITOR_SYSMON_CORE_DEFS_H_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <inttypes.h>

#include "modules/common/macro.h"
#include "modules/common/log.h"

#ifdef _DEBUG
#define DBG_VAR(t, v) t v
#define DBG_ONLY(args...) do{args; } while(0)
#else
#define DBG_VAR(t, v)
#define DBG_ONLY(args...)
#endif

#define DO_ONCE(args...) do {\
  static int __##__FILE__##__LINE__ = 0;  \
  if (__##__FILE__##__LINE__ ++ < 1) {args;} } while(0)

// Annotations {

#ifndef MUST_USE
/// Indicating the caller must check the return value of the annotated function.
#define MUST_USE
#endif

/// Indicating access to the given (data) member is lock-protected by the given
/// lock.
#define XLOCK_BY(lock)

/// Indicating this function will acquire the given exclusive lock(s).
#define ACQ_LOCK(locks...)

/// Indicating this function should only be called when the given exclusive
/// lock(s) is/are locked.
#define WITH_LOCK(locks...)

// }

#endif  // MODULES_MONITOR_SYSMON_CORE_DEFS_H_
