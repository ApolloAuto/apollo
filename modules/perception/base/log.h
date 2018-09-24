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
#ifndef PERCEPTION_BASE_LOG_H_
#define PERCEPTION_BASE_LOG_H_

#include "glog/logging.h"
#include "modules/perception/base/options_define.h"

// 000-099 reserve for common error
#define COMMON_MEMORY_ERROR 000            // memory error, e.g. visit null
#define COMMON_SENSOR_ERROR 001            // sensor type/id was not defined
#define COMMON_INVALID_FUNCTION_PARAM 002  // invalid function parameter

#ifdef PERCEPTION_USE_CYBERTRON
/**************************** USE CYBERTRON LOG ******************************/
#include "cybertron/common/logger.h"

#else
/******************************** USE GLOG ***********************************/
#define LOG_DEBUG DLOG(INFO)
#define LOG_INFO LOG(INFO)
#define LOG_WARN LOG(WARNING)
#define LOG_ERROR LOG(ERROR)
#define LOG_FATAL LOG(FATAL)
#define LOG_V(log_severity) VLOG(log_severity)

// LOG_IF
#define LOG_INFO_IF(cond) LOG_IF(INFO, cond)
#define LOG_ERROR_IF(cond) LOG_IF(ERROR, cond)

// LOG_EVERY_N
#define LOG_INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define LOG_WARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define LOG_ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#endif  // USE_CYBERTRON
#ifndef RETURN_IF_NULL
#define RETURN_IF_NULL(ptr)             \
  if (ptr == nullptr) {                 \
    LOG_WARN << #ptr << " is nullptr."; \
    return;                             \
  }
#endif

#ifndef RETURN_VAL_IF_NULL
#define RETURN_VAL_IF_NULL(ptr, val)    \
  if (ptr == nullptr) {                 \
    LOG_WARN << #ptr << " is nullptr."; \
    return val;                         \
  }
#endif

#ifndef RETURN_IF
#define RETURN_IF(condition)                  \
  if (condition) {                            \
    LOG_WARN << #condition << " is not met."; \
    return;                                   \
  }
#endif

#ifndef RETURN_VAL_IF
#define RETURN_VAL_IF(condition, val)         \
  if (condition) {                            \
    LOG_WARN << #condition << " is not met."; \
    return val;                               \
  }
#endif

#define GLOG_TIMESTAMP(timestamp) std::to_string(timestamp)

#endif  // PERCEPTION_BASE_LOG_H_
