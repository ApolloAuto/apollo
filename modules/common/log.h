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

/**
 * @file
 */

#ifndef MODULES_COMMON_LOG_H_
#define MODULES_COMMON_LOG_H_

#include "glog/logging.h"
#include "glog/raw_logging.h"

#define ADEBUG VLOG(4) << "[DEBUG] "
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)

// LOG_IF
#define AINFO_IF(cond) LOG_IF(INFO, cond)
#define AERROR_IF(cond) LOG_IF(ERROR, cond)

// LOG_EVERY_N
#define AINFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define AWARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define AERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#endif  // MODULES_COMMON_LOG_H_
