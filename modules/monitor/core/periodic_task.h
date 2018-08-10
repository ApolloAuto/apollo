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

#ifndef MODULES_MONITOR_SYSMON_PERIODIC_TASK_H_
#define MODULES_MONITOR_SYSMON_PERIODIC_TASK_H_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>

#include "modules/common/macro.h"
#include "modules/common/log.h"

#include "defs.h"

namespace apollo {
namespace monitor {
namespace sysmon {

class PeriodicTask;

typedef std::function<void()> TaskFn;

/// # of nano-seconds in a second.
const static uint64_t NANOS_IN_SEC = 1000000000;

/// # of nano-seconds in a micro-second.
const static uint64_t NANOS_IN_USEC = 1000;

/// Base class that for constructing class that does something periodically.
class PeriodicTask {
public:
  PeriodicTask(uint64_t interval_ns)
      : _interval(interval_ns), _stop_flag(false) {
    DBG_ONLY(_run_cnt = 0);
    ADEBUG << "PeriodicTask created with interval " << interval_ns << "ns";
  }

  virtual ~PeriodicTask() {}

  /// Stops task; task may only actually stop when it reaches the next periodic
  /// run time.
  inline void stop() { _stop_flag.store(true); }

  /// Returns a function that can be passed to a thread constructor to start
  /// a thread that runs do_task() periodically per _interval.
  inline std::function<void()> get_runner() {
    return std::bind(&PeriodicTask::run, this);
  }

protected:
  std::chrono::nanoseconds _interval;
  std::atomic<bool> _stop_flag;

  /// Override this function to do something useful.
  virtual void do_task() = 0;

  DBG_VAR(uint64_t, _run_cnt);

private:
  void run();

  DISALLOW_COPY_AND_ASSIGN(PeriodicTask);
};


/// Runs a given task periodically.
class PeriodicRunner: public PeriodicTask {
public:
  PeriodicRunner(uint64_t interval_ns, TaskFn task_fn)
      : PeriodicTask(interval_ns), _task_fn(task_fn) {}

  virtual ~PeriodicRunner() {}

  inline bool has_task() { return true; }

protected:
  TaskFn _task_fn;

  void do_task() final { _task_fn(); }

private:
  DISALLOW_COPY_AND_ASSIGN(PeriodicRunner);
};

}  // namespace sysmon
}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_SYSMON_PERIODIC_TASK_H_
