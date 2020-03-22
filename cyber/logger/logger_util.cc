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

#include "cyber/logger/logger_util.h"

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>

#include "glog/logging.h"

namespace apollo {
namespace cyber {
namespace logger {

static int32_t g_main_thread_pid = getpid();

int32_t GetMainThreadPid() { return g_main_thread_pid; }

bool PidHasChanged() {
  int32_t pid = getpid();
  if (g_main_thread_pid == pid) {
    return false;
  }
  g_main_thread_pid = pid;
  return true;
}

}  // namespace logger
}  // namespace cyber
}  // namespace apollo
