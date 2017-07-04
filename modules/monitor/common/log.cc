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

#include "log.h"

#include <stdio.h>

namespace apollo {
namespace platform {
namespace log {

void platform_log_printf(int, const char *format, va_list ap)
{
  vprintf(format, ap);
}


void platform_log_write(const LogModule *mod, int priority, const char *format, ...)
{
  va_list args;
  va_start(args, format);
  mod->log_fn(priority, format, args);
  va_end(args);
}

void init_syslog(const char *tag)
{
  openlog(tag, LOG_CONS | LOG_PID | LOG_DAEMON, LOG_USER);
}

}  // namespace log
}  // namespace platform
}  // namespace apollo
