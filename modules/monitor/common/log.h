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

#ifndef MODULES_PLATFORM_LOG_H_
#define MODULES_PLATFORM_LOG_H_

#include <stdarg.h>
#include <syslog.h>

#include "common_defs.h"

/**
 * @namespace apollo::platform::log
 * @brief apollo::platform::log
 */
namespace apollo {
namespace platform {

/// Logging support.
/// Features: (1) modules -- different modules may have different logging levels and log writers,
/// very handy for debugging;
/// (2) separation of log writer from logging logic;
/// (3) debug logging that can be completely compiled out.
///
/// @todo: log prefix
namespace log {

// Use log levels as defined in syslog.
/// Log levels.
enum {
  LVL_EMERG = LOG_EMERG,
  LVL_ALERT = LOG_ALERT,
  LVL_CRIT = LOG_CRIT,
  LVL_ERR = LOG_ERR,
  LVL_WARN = LOG_WARNING,
  LVL_NOTICE = LOG_NOTICE,
  LVL_INFO = LOG_INFO,
  LVL_DBG = LOG_DEBUG,
  // syslog levels defined up to LOG_DEBUG
  // Add extra debug levels.
  DBG_VERBOSE = LOG_DEBUG + 1,
  DBG_XTRA1,
  DBG_XTRA2
};

/// Type of log writer function.
typedef void (LogFn)(int priority, const char *format, va_list ap);

struct LogModule {
  const char *name;
  /// Log level of this module.
  int log_lvl;
  /// Debug log level of this module.
  int dbg_lvl;
  /// Log function to use for this module.
  LogFn *log_fn;

  /// Sets log level of this module.
  /// @return old log level.
  MEMBER_SET_AND_RETURN(int, log_lvl);

  /// Sets debug log level of this module.
  /// @return old log level.
  MEMBER_SET_AND_RETURN(int, dbg_lvl);

  /// Sets log function of this module.
  /// @return old log function.
  MEMBER_SET_AND_RETURN(LogFn*, log_fn);
};

/// Log function that just uses plain printf.
void platform_log_printf(int, const char *format, va_list ap);

/// Log function that writes to syslog.
void platform_log_write(const LogModule *mod, int priority, const char *format, ...);

/// Init syslog for logging to syslog.
/// @param tag tag (name) to use in syslog.
void init_syslog(const char *tag = "apollo-pl");

/// Use this to log to syslog.
#define PLATFORM_LOG_FN_SYSLOG (&vsyslog)

}  // namespace log


/// Logging function.
#define PLATFORM_LOG(mod, lvl, fmt, args...)  \
if (lvl <= (mod)->log_lvl) {  \
    platform_log_write((mod), lvl, "%s:%d@%s:%d: " fmt "\n", \
        (mod)->name, lvl, __FILE__, __LINE__, ##args); }


#ifdef DEBUG
/// Debug logging function.
#define PLATFORM_DBG(mod, lvl, fmt, args...)  \
if (lvl <= (mod)->dbg_lvl) {  \
    platform_log_write((mod), lvl, "%s:%d@%s:%d: " fmt "\n", \
        (mod)->name, lvl, __FILE__, __LINE__, ##args); }
#else
// PLATFORM_DBG() has 0-overhead in non-debug mode.
#define PLATFORM_DBG(args...) do{}while(0)
#endif

}  // namespace platform
}  // namespace apollo

#endif  // MODULES_PLATFORM_LOG_H_
