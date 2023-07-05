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

/**
 * @file
 */

#ifndef CYBER_LOGGER_LOGGER_UTIL_H_
#define CYBER_LOGGER_LOGGER_UTIL_H_

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

#include "cyber/common/global_data.h"

namespace apollo {
namespace cyber {
namespace logger {

inline int64_t CycleClock_Now() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return static_cast<int64_t>(tv.tv_sec) * 1000000 + tv.tv_usec;
}

inline int64_t UsecToCycles(int64_t usec) { return usec; }

static inline void GetHostName(std::string* hostname) {
  struct utsname buf;
  if (0 != uname(&buf)) {
    // ensure null termination on failure
    *buf.nodename = '\0';
  }
  *hostname = buf.nodename;
}

int32_t GetMainThreadPid();

bool PidHasChanged();

inline int32_t MaxLogSize() {
  return (FLAGS_max_log_size > 0 ? FLAGS_max_log_size : 1);
}

inline void FindModuleName(std::string* log_message, std::string* module_name) {
  auto lpos = log_message->find(LEFT_BRACKET);
  if (lpos != std::string::npos) {
    auto rpos = log_message->find(RIGHT_BRACKET, lpos);
    if (rpos != std::string::npos) {
      module_name->assign(*log_message, lpos + 1, rpos - lpos - 1);
      auto cut_length = rpos - lpos + 1;
      log_message->erase(lpos, cut_length);
    }
  }
  if (module_name->empty()) {
    CHECK_NOTNULL(common::GlobalData::Instance());
    *module_name = common::GlobalData::Instance()->ProcessGroup();
  }
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_LOGGER_LOGGER_UTIL_H_
