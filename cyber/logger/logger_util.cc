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

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <time.h>
#include <unistd.h>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include "cyber/common/global_data.h"
#include "cyber/logger/logger_util.h"
#include "glog/logging.h"

namespace apollo {
namespace cyber {
namespace logger {

namespace {
static int32_t g_main_thread_pid = getpid();
}

int32_t GetMainThreadPid() { return g_main_thread_pid; }

bool PidHasChanged() {
  int32_t pid = getpid();
  if (g_main_thread_pid == pid) {
    return false;
  }
  g_main_thread_pid = pid;
  return true;
}

const std::vector<std::string>& GetLoggingDirectories() {
  static std::vector<std::string> logging_directories_list;
  if (logging_directories_list.empty()) {
    if (!FLAGS_log_dir.empty()) {
      logging_directories_list.emplace_back(FLAGS_log_dir.c_str());
    } else {
      logging_directories_list.emplace_back("./");
    }
  }
  return logging_directories_list;
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo
